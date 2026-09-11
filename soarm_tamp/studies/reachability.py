#!/usr/bin/env python3
"""Reproduce the SO-101 measurements that the scene geometry rests on.

RUNS IN THE PLANNING CONTAINER (needs pinocchio):

    scripts/hpp_container.sh exec \
        'python3 -m soarm_tamp.studies.reachability'

Every number in ``soarm_tamp/geometry.py`` came from here. It is kept
runnable so those numbers can be re-derived rather than trusted — they
decide where the cube goes and how deep the gripper closes, and a stale
constant would be invisible until the arm hit something.

Three measurements:

1. **Top-down reach.** A forward-kinematics sweep over random arm
   configurations, asking where the TCP can be with its approach axis
   near-vertical. This is what says the cube belongs in a 0.10-0.30 m
   annulus. It is done by forward sampling rather than IK deliberately:
   an IK failure is ambiguous (unreachable, or a solver that stalled?),
   and the first attempt at this study reported 0% reachable purely
   because of a badly conditioned damped-least-squares step.

2. **Yaw freedom.** At each candidate cube spot, how much of the gripper's
   rotation about the vertical is actually attainable. ~170 deg, which is
   why the grasp handle can leave that DOF free and why the arm is not
   over-constrained by a top-down grasp.

3. **Jaw geometry.** Opening and centerline offset versus the `gripper`
   joint angle, measured from the STL meshes. Gives CUBE_SIZE_M, the
   grasp-frame X offset, and the open/closed commands.
"""

from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin

URDF_DIR = Path("/home/thanhndv212/devel/soarm-ws/SO-ARM100/Simulation/SO101")
URDF = URDF_DIR / "so101_new_calib.urdf"
TCP_FRAME = "gripper_frame_link"


def _stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    n = struct.unpack("<I", raw[80:84])[0]
    tri = np.frombuffer(raw, dtype=np.uint8, count=n * 50, offset=84).reshape(n, 50)
    return np.unique(tri[:, 12:48].copy().view("<f4").reshape(-1, 3), axis=0).astype(
        float
    )


def _link_geoms(link: str) -> list[tuple[pin.SE3, np.ndarray]]:
    """Mesh vertices of a link, each with its own <origin> applied.

    Skipping that origin is a real trap: it silently puts every mesh at
    the link frame and made an early version of this study report the jaws
    nowhere near the TCP.
    """
    root = ET.parse(URDF).getroot()
    out = []
    for lk in root.findall("link"):
        if lk.get("name") != link:
            continue
        for c in list(lk.findall("collision")) or list(lk.findall("visual")):
            mesh = c.find("geometry/mesh")
            if mesh is None:
                continue
            o = c.find("origin")
            xyz = np.array(
                [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())]
            )
            rpy = np.array(
                [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())]
            )
            out.append((pin.SE3(pin.rpy.rpyToMatrix(*rpy), xyz), _stl(URDF_DIR / mesh.get("filename"))))
    return out


def topdown_reach(model, data, fid, n: int = 300_000, seed: int = 0) -> None:
    rng = np.random.default_rng(seed)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    Q = rng.uniform(lo[0:5], hi[0:5], size=(n, 5))
    P = np.empty((n, 3))
    Z = np.empty((n, 3))
    q = pin.neutral(model)
    for i in range(n):
        q[0:5] = Q[i]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        P[i] = data.oMf[fid].translation
        Z[i] = data.oMf[fid].rotation[:, 2]
    tilt = np.degrees(np.arccos(np.clip(-Z[:, 2], -1, 1)))
    band = (P[:, 2] > 0.015) & (P[:, 2] < 0.065)
    print(f"\n[1] Top-down reach  ({n} samples)")
    print(f"    TCP envelope: x[{P[:,0].min():.2f},{P[:,0].max():.2f}] "
          f"y[{P[:,1].min():.2f},{P[:,1].max():.2f}] z[{P[:,2].min():.2f},{P[:,2].max():.2f}]")
    for tol in (5, 10, 20):
        sel = band & (tilt < tol)
        if not sel.any():
            continue
        r = np.hypot(P[sel, 0], P[sel, 1])
        print(f"    within {tol:2d} deg of vertical, table height: "
              f"radius [{r.min():.3f}, {r.max():.3f}] m  ({sel.sum()} pts)")


def yaw_freedom(model, data, fid, n: int = 600_000, seed: int = 1) -> None:
    rng = np.random.default_rng(seed)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    Q = rng.uniform(lo[0:5], hi[0:5], size=(n, 5))
    P = np.empty((n, 3))
    Z = np.empty((n, 3))
    X = np.empty((n, 3))
    q = pin.neutral(model)
    for i in range(n):
        q[0:5] = Q[i]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        R = data.oMf[fid].rotation
        P[i], Z[i], X[i] = data.oMf[fid].translation, R[:, 2], R[:, 0]
    tilt = np.degrees(np.arccos(np.clip(-Z[:, 2], -1, 1)))
    yaw = np.degrees(np.arctan2(X[:, 1], X[:, 0]))
    band = (P[:, 2] > 0.015) & (P[:, 2] < 0.065) & (tilt < 10)
    print(f"\n[2] Yaw freedom at candidate cube spots  ({n} samples)")
    for tx, ty in [(0.22, -0.10), (0.22, 0.10), (0.25, 0.0), (0.20, 0.0)]:
        sel = band & (np.hypot(P[:, 0] - tx, P[:, 1] - ty) < 0.02)
        if sel.sum() < 8:
            print(f"    ({tx:+.2f},{ty:+.2f}) too few samples")
            continue
        yy = np.sort(yaw[sel] % 180.0)
        print(f"    ({tx:+.2f},{ty:+.2f})  yaw spread {yy.max()-yy.min():5.1f} deg "
              f"over {sel.sum():4d} samples")


def jaw_geometry(model, data) -> None:
    jaw_f = model.getFrameId("moving_jaw_so101_v1_link")
    grip_f = model.getFrameId("gripper_link")
    tcp_f = model.getFrameId(TCP_FRAME)
    GJ, GG = _link_geoms("moving_jaw_so101_v1_link"), _link_geoms("gripper_link")
    q = pin.neutral(model)
    print("\n[3] Jaw geometry (inner faces, mid-finger band z in [-35,-10] mm)")
    print(f"    {'angle':>8}{'opening':>10}{'centerline x':>14}")
    for deg in (-10, -5, 0, 5, 10, 15, 20, 30):
        q[5] = np.radians(deg)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        Mt = data.oMf[tcp_f]

        def band(fid, geoms):
            P = np.vstack([
                ((data.oMf[fid] * t).rotation @ V.T).T + (data.oMf[fid] * t).translation
                for t, V in geoms
            ])
            L = (Mt.rotation.T @ (P - Mt.translation).T).T
            return L[(L[:, 2] > -0.035) & (L[:, 2] < -0.010) & (np.abs(L[:, 1]) < 0.02)]

        A, B = band(jaw_f, GJ), band(grip_f, GG)
        if len(A) < 5 or len(B) < 5:
            print(f"    {deg:>6}deg  (jaw out of band)")
            continue
        a, b = A[:, 0].max(), B[:, 0].min()
        print(f"    {deg:>6}deg{(b-a)*1000:>9.1f}mm{((a+b)/2)*1000:>12.1f}mm")


def main() -> None:
    model = pin.buildModelFromUrdf(str(URDF))
    data = model.createData()
    fid = model.getFrameId(TCP_FRAME)
    print("=" * 70)
    print("SO-101 geometry study — reproduces soarm_tamp/geometry.py")
    print("=" * 70)
    topdown_reach(model, data, fid)
    yaw_freedom(model, data, fid)
    jaw_geometry(model, data)
    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()
