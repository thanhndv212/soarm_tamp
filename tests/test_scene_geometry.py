"""Regression checks for the nominal cube scene and generated grasp frames."""

import copy
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from soarm_tamp.build_assets import _write_cube, _write_robot_srdf, _write_table_srdf
from soarm_tamp.geometry import (
    CUBE_SIZE_M,
    FINGERTIP_BELOW_TCP_M,
    FINGERTIP_TABLE_CLEARANCE_M,
    GRASP_CENTERLINE_X_M,
    GRASP_DEPTH_Z_M,
    PLACE_XY,
    PLANNING_JAW_DEG,
    checked_planning_jaw,
    compare_scene,
    scene_fingerprint,
    validate_nominal_scene,
)


def nominal_config() -> dict:
    return {
        "joint_groups": {
            "SO101": [{"joint": "so101/gripper", "initial": math.radians(20)}]
        },
        "objects": {
            "cube": {
                "initial_pose_xyzquat": [0.22, -0.10, 0.015, 0, 0, 0, 1]
            }
        },
    }


class SceneGeometryTest(unittest.TestCase):
    def test_nominal_config_matches_geometry(self) -> None:
        validate_nominal_scene(nominal_config(), "test-config.yaml")

    def test_old_cube_height_is_rejected(self) -> None:
        config = copy.deepcopy(nominal_config())
        config["objects"]["cube"]["initial_pose_xyzquat"][2] = 0.0125
        with self.assertRaisesRegex(ValueError, "test-config.yaml: cube initial z"):
            validate_nominal_scene(config, "test-config.yaml")

    def test_old_jaw_angle_is_rejected(self) -> None:
        config = copy.deepcopy(nominal_config())
        config["joint_groups"]["SO101"][0]["initial"] = math.radians(10)
        with self.assertRaisesRegex(ValueError, "test-config.yaml: frozen gripper"):
            validate_nominal_scene(config, "test-config.yaml")

    def test_measured_start_jaw_must_match_grasp_frame(self) -> None:
        expected = math.radians(PLANNING_JAW_DEG)
        self.assertEqual(checked_planning_jaw(expected + math.radians(0.5), "start.json"), expected)
        with self.assertRaisesRegex(ValueError, "start.json: measured gripper"):
            checked_planning_jaw(math.radians(10), "start.json")

    def test_generated_cube_and_grasp_frames(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            robot_urdf = root / "robot.urdf"
            robot_urdf.write_text('<robot name="so101"/>')
            cube_urdf = root / "cube.urdf"
            cube_srdf = root / "cube.srdf"
            robot_srdf = root / "so101.srdf"
            table_srdf = root / "table.srdf"

            _write_cube(cube_urdf, cube_srdf)
            _write_robot_srdf(robot_srdf, robot_urdf)
            _write_table_srdf(table_srdf)

            cube = ET.parse(cube_urdf).getroot()
            for kind in ("visual", "collision"):
                size = cube.find(f"./link/{kind}/geometry/box").get("size")
                self.assertEqual([float(value) for value in size.split()], [CUBE_SIZE_M] * 3)
            mass = float(cube.find("./link/inertial/mass").get("value"))
            inertia = cube.find("./link/inertial/inertia")
            for axis in ("ixx", "iyy", "izz"):
                self.assertAlmostEqual(float(inertia.get(axis)), mass * CUBE_SIZE_M**2 / 6)

            cube_frames = ET.parse(cube_srdf).getroot()
            top = cube_frames.find("./handle[@name='top']/position")
            foot = cube_frames.find("./gripper[@name='foot']/position")
            self.assertEqual([float(value) for value in top.get("xyz").split()], [0, 0, 0])
            self.assertAlmostEqual(float(foot.get("xyz").split()[2]), -CUBE_SIZE_M / 2)

            robot_frames = ET.parse(robot_srdf).getroot()
            grasp = robot_frames.find("./gripper[@name='grasp']/position")
            grasp_xyz = [float(value) for value in grasp.get("xyz").split()]
            self.assertAlmostEqual(grasp_xyz[0], GRASP_CENTERLINE_X_M)
            self.assertAlmostEqual(grasp_xyz[2], GRASP_DEPTH_Z_M)
            self.assertAlmostEqual(
                CUBE_SIZE_M / 2 + grasp_xyz[2] - FINGERTIP_BELOW_TCP_M,
                FINGERTIP_TABLE_CLEARANCE_M,
            )

            table_frames = ET.parse(table_srdf).getroot()
            spot = table_frames.find("./handle[@name='spot_b']/position")
            spot_xyz = [float(value) for value in spot.get("xyz").split()]
            self.assertAlmostEqual(spot_xyz[0], PLACE_XY[0])
            self.assertAlmostEqual(spot_xyz[1], PLACE_XY[1])
            self.assertAlmostEqual(spot_xyz[2], 0)
            self.assertAlmostEqual(
                spot_xyz[2] - float(foot.get("xyz").split()[2]),
                CUBE_SIZE_M / 2,
            )

    def test_scene_fingerprint_invalidates_old_runs(self) -> None:
        fingerprint = scene_fingerprint()
        self.assertEqual(fingerprint["cube_rest_center_z_m"], CUBE_SIZE_M / 2)
        self.assertEqual(PLANNING_JAW_DEG, 20.0)
        del fingerprint["cube_rest_center_z_m"]
        self.assertTrue(compare_scene(fingerprint))


if __name__ == "__main__":
    unittest.main()
