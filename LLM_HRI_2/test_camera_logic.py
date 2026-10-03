import copy
import time
import unittest
from unittest.mock import Mock

import numpy as np

from skill_executor import execute_plan
from vision import (OBJECTS, ZONES, occupancy, choose_temporary, point_image,
                    require_fresh, analyze, CAMERA_POSITION)


def render_scene(blue_position):
    k = np.array([617.24, 0, 400, 0, 617.24, 300, 0, 0, 1])
    rays = point_image(np.ones((600, 800)), k) - CAMERA_POSITION
    depth = (-CAMERA_POSITION[2] / rays[:, :, 2]).astype(np.float32)
    rgb = np.full((600, 800, 3), (185, 153, 113), np.uint8)
    surfaces = [
        ((.35, -.16), .12, .002, (0, 255, 255)),
        ((.35, 0), .12, .002, (255, 122, 8)),
        ((.35, .16), .12, .002, (255, 0, 166)),
        ((.205, -.145), .05, .05, (255, 8, 8)),
        ((.215, .015), .05, .05, (255, 217, 5)),
        (blue_position, .05, .05, (5, 38, 255)),
        ((.225, .18), .05, .05, (5, 255, 8)),
        ((.35, .16), .05, .05, (153, 5, 255)),
    ]
    for (x, y), size, height, color in surfaces:
        distance = (height - CAMERA_POSITION[2]) / rays[:, :, 2]
        world = rays * distance[..., None] + CAMERA_POSITION
        mask = ((np.abs(world[:, :, 0]-x) < size/2)
                & (np.abs(world[:, :, 1]-y) < size/2)
                & (distance < depth) & (distance > 0))
        depth[mask] = distance[mask]
        rgb[mask] = color
    return rgb, depth, k


def state():
    objects = {name: {"position": [0.22, y, .025]} for name, y in zip(
        OBJECTS, [-.16, 0, .16, -.28, .28])}
    objects["blue_cube"]["position"] = [.36, 0, .025]
    zones = {name: {"position": [.36, y, .001]} for name, y in zip(ZONES, [-.16, 0, .16])}
    return {"source": "rgbd_camera", "observed_at": time.monotonic(), "complete": True,
            "objects": objects, "zones": occupancy(objects, zones, True),
            "free_positions": [[.26, .29, .025]]}


class CameraLogicTests(unittest.TestCase):
    def test_rgbd_measures_moved_cubes_and_zone_occupancy(self):
        occupied, _ = analyze(*render_scene((.35, 0)))
        cleared, _ = analyze(*render_scene((.26, -.28)))
        self.assertTrue(occupied["complete"])
        self.assertTrue(cleared["complete"])
        self.assertEqual(occupied["zones"]["zone_b"]["occupants"], ["blue_cube"])
        self.assertEqual(cleared["zones"]["zone_b"]["status"], "FREE")
        np.testing.assert_allclose(cleared["objects"]["blue_cube"]["position"],
                                   [.26, -.28, .025], atol=.002)
        np.testing.assert_allclose(cleared["objects"]["red_cube"]["position"],
                                   [.205, -.145, .025], atol=.002)
        self.assertTrue(cleared["free_positions"])

    def test_missing_rgb_object_does_not_mean_free_zone(self):
        rgb, depth, k = render_scene((.35, 0))
        blue = (rgb[:,:,2] == 255) & (rgb[:,:,0] == 5)
        rgb[blue] = 0
        observation, _ = analyze(rgb, depth, k)
        self.assertFalse(observation["complete"])
        self.assertEqual(observation["zones"]["zone_b"]["status"], "UNKNOWN")
        self.assertEqual(observation["free_positions"], [])

    def test_occupied_zone_and_missing_detection(self):
        scene = state()
        self.assertEqual(scene["zones"]["zone_b"]["occupants"], ["blue_cube"])
        self.assertEqual(scene["zones"]["zone_a"]["status"], "FREE")
        unknown = occupancy({}, scene["zones"], False)
        self.assertTrue(all(zone["status"] == "UNKNOWN" for zone in unknown.values()))

    def test_reject_stale_camera(self):
        scene = state()
        scene["observed_at"] -= 10
        with self.assertRaisesRegex(RuntimeError, "STALE"):
            require_fresh(scene)

    def test_no_space_or_incomplete_fails(self):
        scene = state()
        scene["free_positions"] = []
        with self.assertRaisesRegex(RuntimeError, "NO_FREE"):
            choose_temporary(scene)
        scene["complete"] = False
        with self.assertRaisesRegex(RuntimeError, "INCOMPLETE"):
            choose_temporary(scene)

    def test_projection_uses_depth(self):
        k = [600, 0, 0, 0, 600, 0, 0, 0, 1]
        a = point_image(np.array([[1.0]]), k)
        b = point_image(np.array([[0.8]]), k)
        self.assertGreater(np.linalg.norm(a-b), .19)

    def test_blocker_moved_before_requested_cube(self):
        initial = state()
        cleared = copy.deepcopy(initial)
        cleared["objects"]["blue_cube"]["position"] = [.26, .29, .025]
        cleared["zones"] = occupancy(cleared["objects"], cleared["zones"], True)
        finished = copy.deepcopy(cleared)
        finished["objects"]["red_cube"]["position"] = [.36, 0, .025]
        finished["zones"] = occupancy(finished["objects"], finished["zones"], True)
        robot = Mock()
        robot.held_object = None
        robot.sync_environment.side_effect = [initial, cleared, cleared, finished]
        robot.pick.return_value = robot.place.return_value = True
        plan = {"plan": [{"skill": "pick", "object": "red_cube"},
                         {"skill": "place", "object": "red_cube", "zone": "zone_b"}]}
        result = execute_plan(plan, robot=robot)
        self.assertEqual(result["status"], "SUCCESS")
        actions = [(c[0], c[1]) for c in robot.mock_calls if c[0] in ("pick", "place")]
        self.assertEqual(actions, [("pick", ("blue_cube",)),
            ("place", ("blue_cube", "temporary_position")),
            ("pick", ("red_cube",)), ("place", ("red_cube", "zone_b"))])

    def test_no_space_does_not_pick(self):
        robot = Mock()
        robot.held_object = None
        observation = state()
        observation["free_positions"] = []
        robot.sync_environment.return_value = observation
        plan = {"plan": [{"skill": "pick", "object": "red_cube"},
                         {"skill": "place", "object": "red_cube", "zone": "zone_b"}]}
        result = execute_plan(plan, robot=robot)
        self.assertEqual(result["status"], "FAILED")
        robot.pick.assert_not_called()

    def test_unpaired_pick_does_not_move(self):
        robot = Mock()
        self.assertEqual(execute_plan({"plan": [{"skill": "pick", "object": "red_cube"}]},
                                     robot=robot)["status"], "INVALID_PLAN")
        robot.pick.assert_not_called()


if __name__ == "__main__":
    unittest.main()
