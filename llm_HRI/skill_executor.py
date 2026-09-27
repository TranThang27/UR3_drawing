import re

from llm.semantic_parse import PlanError, validate_plan


PLANNING_ERROR_CODES = {-1, -2, -10, -11, -12, -13, -14, -16, -31}


def _failure_record(record, error):
    message = str(error)
    record["status"] = "FAILED"
    match = re.search(r"(?:error code|execution failed:)\s*(-?\d+)", message)
    if match:
        code = int(match.group(1))
        record["moveit_error_code"] = code
        if code in PLANNING_ERROR_CODES:
            record["status"] = "PLANNING_FAILED"
    elif (
        message.startswith("Cartesian path")
        or "IK branch" in message
        or message.startswith("Cartesian segment")
    ):
        record["status"] = "PLANNING_FAILED"
    record["message"] = message


def execute_plan(plan, skills=None, on_step=None):
    try:
        plan = validate_plan(plan)
    except PlanError as exc:
        return {"status": exc.status, "message": str(exc), "steps": []}
    if skills is None:
        from action import home, pick, place

        skills = {"home": home, "pick": pick, "place": place}
    results = []
    for index, step in enumerate(plan["plan"]):
        record = {"index": index, **step}
        try:
            arguments = {key: value for key, value in step.items() if key != "skill"}
            success = skills[step["skill"]](**arguments)
            record["status"] = "SUCCESS" if success is True else "FAILED"
        except Exception as exc:
            _failure_record(record, exc)
        results.append(record)
        if on_step:
            on_step(dict(record))
        if record["status"] != "SUCCESS":
            return {"status": record["status"], "steps": results}
    return {"status": "SUCCESS", "steps": results}
