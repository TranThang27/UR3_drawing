import contextlib
import io
import json
import unittest
from unittest.mock import Mock, patch

from llm.semantic_parse import PlanError, validate_plan
from main import main, process_command
from skill_executor import execute_plan

PLAN = {
    "plan": [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]
}


class LanguageTests(unittest.TestCase):
    def test_three_cube_demo_preview(self):
        output = io.StringIO()
        with (
            patch("sys.argv", ["main.py", "--demo", "--dry-run"]),
            patch("main.execute_plan") as run,
            contextlib.redirect_stdout(output),
        ):
            main()
        plan = json.loads(output.getvalue())["plan"]
        validate_plan({"plan": plan})
        placements = [
            (step["object"], step["zone"])
            for step in plan
            if step["skill"] == "place"
        ]
        self.assertEqual(
            placements,
            [
                ("red_cube", "zone_a"),
                ("blue_cube", "zone_b"),
                ("yellow_cube", "zone_c"),
            ],
        )
        self.assertEqual(plan[0], {"skill": "home"})
        self.assertEqual(plan[-1], {"skill": "home"})
        run.assert_not_called()

    def test_success_dispatch(self):
        skills = {name: Mock(return_value=True) for name in ("pick", "place", "home")}
        result = execute_plan(PLAN, skills)
        self.assertEqual(result["status"], "SUCCESS")
        skills["pick"].assert_called_once_with(object="red_cube")
        skills["place"].assert_called_once_with(object="red_cube", zone="zone_b")
        skills["home"].assert_called_once_with()

    def test_validate_all_before_motion(self):
        pick = Mock()
        invalid_plan = {
            "plan": [
                PLAN["plan"][0],
                {"skill": "place", "object": "red_cube", "zone": "zone_d"},
            ]
        }
        result = execute_plan(invalid_plan, {"pick": pick})
        self.assertEqual(result["status"], "INVALID_ZONE")
        pick.assert_not_called()

    def test_invalid_object(self):
        result = execute_plan(
            {"plan": [{"skill": "pick", "object": "green_cube"}]}
        )
        self.assertEqual(result["status"], "INVALID_OBJECT")

    def test_invalid_sequences_and_extra_fields(self):
        invalid_plans = (
            [PLAN["plan"][1]],
            [PLAN["plan"][0], PLAN["plan"][0]],
            [{"skill": "home", "code": "anything"}],
            [{"skill": "exec"}],
            [],
        )
        for plan in invalid_plans:
            with self.assertRaises(PlanError):
                validate_plan({"plan": plan})

    def test_failure_stops_plan(self):
        failures = (
            ("MoveIt failed with error code -1", "PLANNING_FAILED"),
            ("MoveIt failed with error code -4", "FAILED"),
            ("Cartesian path only 50% complete", "PLANNING_FAILED"),
        )
        for error, expected in failures:
            pick = Mock(side_effect=RuntimeError(error))
            place = Mock()
            result = execute_plan(PLAN, {"pick": pick, "place": place})
            self.assertEqual(result["status"], expected)
            self.assertEqual(len(result["steps"]), 1)
            place.assert_not_called()

    def test_dry_run_never_executes(self):
        with patch("main.parse_command", return_value=PLAN), patch(
            "main.execute_plan"
        ) as run:
            result = process_command("Move the red cube to zone B.", True)
            self.assertEqual(result["status"], "PLAN_READY")
            run.assert_not_called()

    def test_api_failure_never_executes(self):
        with patch(
            "main.parse_command",
            side_effect=PlanError("LLM_FAILED", "Unavailable"),
        ), patch("main.execute_plan") as run:
            result = process_command("Move the red cube to zone B.")
            self.assertEqual(result["status"], "LLM_FAILED")
            run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
