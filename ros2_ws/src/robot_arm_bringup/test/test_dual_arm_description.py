"""Verify that TF placement and arm geometry agree with the IK inputs."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

_package = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    "dual_arm_launch", _package / "launch" / "dual_arm_harvest.launch.py")
_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_launch)
_source = _package.parent


def placement_from_origin(origin):
    x, y, z = map(float, origin.attrib["xyz"].split())
    roll, pitch, yaw = map(float, origin.attrib["rpy"].split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0, 0, 0, 1],
    ]


class DualArmDescription(unittest.TestCase):
    def setUp(self):
        self.single = (_source / "robot_arm_description" / "urdf" / "imported"
                       / "arm_urdf_v1.urdf").read_text()
        self.transforms = _launch._read_transforms(
            _source / "robot_vision" / "config" / "camera_to_bases.json")
        self.dual = ET.fromstring(_launch._dual_arm_description(self.single, self.transforms))

    def test_both_models_match_driver_names_and_have_connected_frames(self):
        links = [node.attrib["name"] for node in self.dual.findall("link")]
        joints = self.dual.findall("joint")
        names = [node.attrib["name"] for node in joints]
        self.assertEqual(len(links), len(set(links)))
        self.assertEqual(len(names), len(set(names)))
        parents = {}
        for joint in joints:
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            self.assertIn(parent, links)
            self.assertIn(child, links)
            self.assertNotIn(child, parents)
            parents[child] = parent
        self.assertEqual(set(links) - set(parents), {"camera_color_optical_frame"})
        for link in links:
            chain = set()
            while link in parents:
                self.assertNotIn(link, chain)
                chain.add(link)
                link = parents[link]
            self.assertEqual(link, "camera_color_optical_frame")
        driver = yaml.safe_load((_source / "robot_arm_driver" / "config"
                                / "dual_arm_driver.yaml").read_text())
        arm_names = set(driver["robot_arm_driver"]["ros__parameters"]["joint_names"])
        arm_names -= {"left_gripper", "right_gripper"}
        self.assertEqual({j.attrib["name"] for j in joints
                          if j.attrib["type"] != "fixed"}, arm_names)
        for side in ("left", "right"):
            self.assertIn(f"{side}_base_link", links)
            self.assertIn(f"{side}_tcp_link", links)

    def test_each_local_ik_chain_preserves_original_geometry_and_meshes(self):
        original = ET.fromstring(self.single)
        for side in ("left", "right"):
            for joint in original.findall("joint"):
                clone = self.dual.find(f"joint[@name='{side}_{joint.attrib['name']}']")
                for tag in ("origin", "axis", "limit"):
                    expected = joint.find(tag)
                    if expected is not None:
                        self.assertEqual(clone.find(tag).attrib, expected.attrib)
                for tag in ("parent", "child"):
                    self.assertEqual(clone.find(tag).attrib["link"],
                                     side + "_" + joint.find(tag).attrib["link"])
            expected_meshes = [e.attrib for e in original.iter("mesh")]
            meshes = [e.attrib for link in self.dual.findall("link")
                      if link.attrib["name"].startswith(side + "_")
                      for e in link.iter("mesh")]
            self.assertEqual(meshes, expected_meshes)

    def assert_mount_matches_transform(self, flat):
        xyz, rpy = _launch._camera_to_base_mount(flat)
        origin = ET.Element("origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
        camera_from_base = placement_from_origin(origin)
        # The camera-parent/base-child URDF placement must undo p_base=T*p_camera.
        for row in range(4):
            for col in range(4):
                value = sum(camera_from_base[row][k] * flat[4 * k + col] for k in range(4))
                self.assertAlmostEqual(value, float(row == col), places=10)

    def test_tf_direction_and_translation_match_json_for_both_arms(self):
        for side in ("left", "right"):
            flat = self.transforms[f"{side}_camera_to_base"]
            self.assert_mount_matches_transform(flat)
            mount = self.dual.find(f"joint[@name='{side}_camera_mount']")
            camera_from_base = placement_from_origin(mount.find("origin"))
            point = [.08, -.12, .53, 1.0]
            base = [sum(flat[4 * i + k] * point[k] for k in range(4)) for i in range(4)]
            actual = [sum(camera_from_base[i][k] * base[k] for k in range(4)) for i in range(4)]
            for value, expected in zip(actual, point):
                self.assertAlmostEqual(value, expected, places=10)

    def test_mount_rpy_for_general_rotation_and_both_pitch_singularities(self):
        for rpy in ((.3, -.4, .7), (.2, math.pi / 2, -.6), (-.5, -math.pi / 2, .9)):
            origin = ET.Element("origin", xyz="0.1 -0.2 0.3", rpy=" ".join(map(str, rpy)))
            matrix = placement_from_origin(origin)
            self.assert_mount_matches_transform([v for row in matrix for v in row])

    def test_invalid_rigid_transform_is_rejected_before_model_publication(self):
        path = _source / "robot_vision" / "config" / "camera_to_bases.json"
        original = json.loads(path.read_text())
        for bad in (float("nan"), 2.0, -1.0):
            document = json.loads(json.dumps(original))
            document["left"]["camera_to_base"] = [
                [bad, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            with tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / "transforms.json"
                config.write_text(json.dumps(document))
                with self.assertRaises(ValueError):
                    _launch._read_transforms(config)


if __name__ == "__main__":
    unittest.main()
