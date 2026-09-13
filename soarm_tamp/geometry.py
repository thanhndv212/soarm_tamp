"""Measured SO-101 gripper and workspace geometry.

Every number here was *measured* from
``SO-ARM100/Simulation/SO101/so101_new_calib.urdf`` and its STL meshes,
not taken from a datasheet — see ``studies/reachability.py``, which
reproduces the measurements. They are kept in one module because both the
asset builder (which bakes them into URDF/SRDF) and the executor (which
commands the real jaw) need the same values, and a silent disagreement
between those two is exactly the bug class that ends with the gripper
closing on air or driving into the table.

Frame conventions
-----------------
``gripper_frame_link`` is the URDF's own TCP frame at the fingertips. In
its local frame:

* ``+Z`` is the approach direction (the way the hand travels toward the
  object). Verified from geometry: the fingers span z in [-51, +6] mm, so
  the open end an object enters through is the +Z end. This matches the
  "gripper local-Z = approach" convention the rest of the long_tamp
  examples use (see ``script/twin/assets/panda_bimanual.srdf``).
* ``+X`` is the jaw-opening direction. The two fingers are *not*
  symmetric about the origin: the moving jaw sits at negative X and the
  fixed finger at positive X, so the true centerline between them is
  offset from the frame origin — see ``GRASP_CENTERLINE_X_M``.
"""

from __future__ import annotations

# --------------------------------------------------------------------------
# Jaw opening as a function of the `gripper` joint angle.
#
# Measured as the clear gap between the inner faces of the two fingers, in
# a mid-finger band (z in [-35, -10] mm of the TCP frame). The centerline
# shifts with the angle because only one jaw moves.
#
#   angle_deg: (opening_mm, centerline_x_mm)
# --------------------------------------------------------------------------
JAW_TABLE: dict[float, tuple[float, float]] = {
    -10.0: (8.2, -2.1),
    -5.0: (14.2, -5.1),
    0.0: (19.8, -7.9),
    5.0: (25.2, -10.6),
    10.0: (29.1, -12.6),
    15.0: (32.8, -14.4),
    20.0: (36.5, -16.3),
    30.0: (43.7, -19.8),
}

# Cube edge length. 30 mm is the cube actually on the bench; the scene was
# built around 25 mm until 2026-09-13. Against JAW_TABLE the jaws close on
# 30 mm at ~+11.2 deg, with range either side for approach clearance and
# squeeze on grip.
CUBE_SIZE_M: float = 0.030

# The jaw angle the *planner* freezes the gripper at. Held wider than the
# cube (36.5 mm vs 30 mm) so the modelled fingers clear it on approach; the
# real open/close happens at execution time and is not part of the plan.
# See GRIPPER_OPEN_DEG / GRIPPER_CLOSED_DEG below.
#
# Must be a key of JAW_TABLE — GRASP_CENTERLINE_X_M indexes it directly.
# At the old 10 deg the opening is 29.1 mm, which is NARROWER than a 30 mm
# cube: every plan would have driven the fingers through it. 15 deg would
# also clear, at 32.8 mm, but only by 1.4 mm a side.
PLANNING_JAW_DEG: float = 20.0

# Lowest point of the whole hand, measured along the approach axis at
# PLANNING_JAW_DEG: the moving jaw's tip, beyond the frame origin. This is
# what actually decides how low the TCP may go.
#
# It is a function of the jaw angle, so it moves whenever PLANNING_JAW_DEG
# does — the jaw tip swings up as the fingers open. Recomputed from the
# URDF meshes in the gripper_frame_link frame; that computation reproduces
# the previously recorded 8.0 mm at 10 deg exactly, which is why its 6.25 mm
# at 20 deg is trusted here.
FINGERTIP_BELOW_TCP_M: float = 0.00625

# How much air to leave under that fingertip at the moment of grasp.
FINGERTIP_TABLE_CLEARANCE_M: float = 0.005

# Grasp frame relative to `gripper_frame_link`, in metres. At grasp the
# object's handle frame is made to coincide with this frame.
#
# X: the jaw centerline at PLANNING_JAW_DEG. Without this offset a grasp
#    commanded at the frame origin would sit ~13 mm off-centre between the
#    jaws — with a 25 mm cube in a 29 mm gap, that is a guaranteed miss.
#
# Z: +Z is the approach direction (world-DOWN for a top-down grasp), so a
#    POSITIVE value puts the grasp point *below* the frame origin and so
#    lifts the whole hand. It is derived, not chosen: the fingertip must
#    clear the table, which fixes the TCP height at FINGERTIP_BELOW_TCP_M
#    + FINGERTIP_TABLE_CLEARANCE_M, and the cube centre sits CUBE_SIZE_M/2
#    above the surface.
#
#    An earlier version seated the cube "mid-finger" at -22.5 mm, reasoning
#    that the fingers span -51..+6 mm so their middle is the natural depth.
#    That is wrong for anything resting on a surface: the fingers extend
#    UPWARD from the tip, so seating an object 22 mm deep needs the tip
#    22 mm below it — here, 10 mm under the table. It failed exactly that
#    way: solver residual 1e-10, result rejected for "Collision between
#    so101/gripper_link_1 and table/base_link_0". Grasping near the tip is
#    no compromise here — the fingers still contact most of the cube's
#    30 mm height.
GRASP_CENTERLINE_X_M: float = JAW_TABLE[PLANNING_JAW_DEG][1] / 1000.0
GRASP_DEPTH_Z_M: float = (
    FINGERTIP_BELOW_TCP_M + FINGERTIP_TABLE_CLEARANCE_M - CUBE_SIZE_M / 2
)

# --------------------------------------------------------------------------
# Execution-time jaw commands (degrees on the `gripper` joint).
#
# CLOSED is deliberately *past* contact on a 30 mm cube (25.2 mm of
# commanded gap vs a 30 mm cube): these are position-controlled serial
# servos, so grip force comes from commanding through the object and
# letting the servo stall against its torque limit. The 4.8 mm of
# over-travel matches what 0 deg gave against the old 25 mm cube; closing
# to 0 deg on a 30 mm cube would be 10.2 mm past contact, twice the squeeze
# and twice the stall current.
# --------------------------------------------------------------------------
GRIPPER_OPEN_DEG: float = 30.0
GRIPPER_CLOSED_DEG: float = 5.0

# --------------------------------------------------------------------------
# Verified top-down workspace (TCP z in [0.015, 0.065] m, approach within
# 10 deg of vertical), from a 600k-sample forward-kinematics sweep. Radial
# distance from the base axis, in metres.
# --------------------------------------------------------------------------
TOPDOWN_RADIUS_MIN_M: float = 0.10
TOPDOWN_RADIUS_MAX_M: float = 0.30



# Where the cube starts (A) and where it must end up (B), in metres, as
# (x, y) on the table. Both sit inside the top-down-reachable annulus
# measured in studies/reachability.py (radius 0.10-0.30 m): |A| = |B| =
# 0.242 m. Defined here rather than in build_assets because they are scene
# geometry, and scene_fingerprint below has to record them.
PICK_XY: tuple[float, float] = (0.22, -0.10)
PLACE_XY: tuple[float, float] = (0.22, 0.10)

# --------------------------------------------------------------------------
# Scene fingerprint.
#
# A manifest is only executable against the scene it was planned in. Change
# the cube size or the grasp frame and every waypoint in an existing run
# refers to geometry that no longer exists — the arm would drive a
# collision-checked path through an object of the wrong size, or grip at an
# offset that misses. Nothing in the manifest itself records which scene it
# came from, so plan.py writes this alongside it and execute.py checks it.
# --------------------------------------------------------------------------
def scene_fingerprint() -> dict:
    """The geometry constants a recorded plan depends on."""
    return {
        "cube_size_m": CUBE_SIZE_M,
        "planning_jaw_deg": PLANNING_JAW_DEG,
        "grasp_centerline_x_m": round(GRASP_CENTERLINE_X_M, 6),
        "grasp_depth_z_m": round(GRASP_DEPTH_Z_M, 6),
        "fingertip_below_tcp_m": FINGERTIP_BELOW_TCP_M,
        "gripper_open_deg": GRIPPER_OPEN_DEG,
        "gripper_closed_deg": GRIPPER_CLOSED_DEG,
        "pick_xy": list(PICK_XY),
        "place_xy": list(PLACE_XY),
    }


def compare_scene(recorded: dict) -> list[str]:
    """Differences between *recorded* and the scene as it is now, if any."""
    current = scene_fingerprint()
    out: list[str] = []
    for key, now in current.items():
        then = recorded.get(key, "<absent>")
        if then != now:
            out.append(f"{key}: planned with {then}, scene is now {now}")
    return out
