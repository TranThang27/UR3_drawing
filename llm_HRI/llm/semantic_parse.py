import json

OBJECTS = {"red_cube", "yellow_cube", "blue_cube"}
ZONES = {"zone_a", "zone_b", "zone_c"}
FIELDS = {
    "pick": {"skill", "object"},
    "place": {"skill", "object", "zone"},
    "home": {"skill"},
}


class PlanError(ValueError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def validate_plan(value):
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError as exc:
            raise PlanError("INVALID_PLAN", "Invalid JSON") from exc
    if not isinstance(value, dict) or set(value) - {"plan", "error"}:
        raise PlanError("INVALID_PLAN", "Expected an object containing plan")
    if value.get("error"):
        raise PlanError("INVALID_PLAN", str(value["error"])[:500])
    steps = value.get("plan")
    if not isinstance(steps, list) or not 1 <= len(steps) <= 20:
        raise PlanError("INVALID_PLAN", "Plan must contain 1 to 20 steps")
    held = None
    for step in steps:
        if not isinstance(step, dict) or not isinstance(step.get("skill"), str):
            raise PlanError("INVALID_PLAN", "Every step needs a skill")
        skill = step["skill"]
        if skill not in FIELDS or set(step) != FIELDS[skill]:
            raise PlanError("INVALID_PLAN", f"Unsupported skill or fields: {skill}")
        if skill != "home":
            if not isinstance(step["object"], str) or step["object"] not in OBJECTS:
                raise PlanError("INVALID_OBJECT", "Expected red_cube, yellow_cube or blue_cube")
        if skill == "pick":
            if held is not None:
                raise PlanError("INVALID_PLAN", "Place the held cube before another pick")
            held = step["object"]
        elif skill == "place":
            if not isinstance(step["zone"], str) or step["zone"] not in ZONES:
                raise PlanError("INVALID_ZONE", "Expected zone_a, zone_b or zone_c")
            if held != step["object"]:
                raise PlanError("INVALID_PLAN", "place must follow pick of the same object")
            held = None
    return {"plan": [dict(step) for step in steps]}
