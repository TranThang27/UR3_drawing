import math

from llm.semantic_parse import PlanError, validate_plan
from vision import choose_temporary, require_fresh


def transfer(robot, obj, zone, emit):
    state = robot.sync_environment()
    require_fresh(state)
    target = state["zones"][zone]
    emit({"skill": "check_zone", "zone": zone, "status": "SUCCESS",
          "occupants": target["occupants"]})
    if target["occupants"] == [obj] and math.dist(
        state["objects"][obj]["position"][:2], target["position"][:2]) < 0.015:
        emit({"skill": "transfer", "object": obj, "zone": zone, "status": "SUCCESS",
              "message": "Already at target, verified by camera"})
        return
    for blocker in target["occupants"]:
        if blocker == obj:
            continue
        temporary = choose_temporary(state)
        candidates = sorted(state["free_positions"], key=lambda p: math.dist(p, temporary))
        temporary = next((point for point in candidates if robot.reachable_temporary(point)), None)
        if temporary is None:
            raise RuntimeError("PLANNING_FAILED: no camera-empty position passes collision-aware IK")
        robot._temporary = temporary
        emit({"skill": "find_temporary_position", "position": temporary, "status": "SUCCESS"})
        if robot.pick(blocker) is not True:
            raise RuntimeError("Pick failed")
        emit({"skill": "pick", "object": blocker, "status": "SUCCESS"})
        if robot.place(blocker, "temporary_position") is not True:
            raise RuntimeError("Temporary placement failed")
        emit({"skill": "place", "object": blocker, "zone": "temporary_position", "status": "SUCCESS"})
        robot.home()
        state = robot.sync_environment()
        if blocker in state["zones"][zone]["occupants"]:
            raise RuntimeError("ZONE_OCCUPIED: camera still sees the blocking cube")
    state = robot.sync_environment()
    if any(name != obj for name in state["zones"][zone]["occupants"]):
        raise RuntimeError("ZONE_OCCUPIED: destination changed during clearance")
    if robot.pick(obj) is not True:
        raise RuntimeError("Pick failed")
    emit({"skill": "pick", "object": obj, "status": "SUCCESS"})
    if robot.place(obj, zone) is not True:
        raise RuntimeError("Place failed")
    robot.home()
    state = robot.sync_environment()
    if state["zones"][zone]["occupants"] != [obj]:
        raise RuntimeError("VERIFICATION_FAILED: camera did not confirm destination")
    emit({"skill": "place", "object": obj, "zone": zone, "status": "SUCCESS"})


def execute_plan(plan, robot=None, on_step=None):
    records = []
    def emit(record):
        record = {"index": len(records), **record}
        records.append(record)
        if on_step:
            on_step(record)
    try:
        steps = validate_plan(plan)["plan"]
        index = 0
        while index < len(steps):
            if steps[index]["skill"] == "home":
                index += 1
                continue
            if (steps[index]["skill"] != "pick" or index + 1 == len(steps)
                    or steps[index + 1]["skill"] != "place"
                    or steps[index]["object"] != steps[index+1]["object"]):
                raise PlanError("INVALID_PLAN", "Camera execution requires adjacent pick/place pairs")
            index += 2
        if robot is None:
            from action.action import _get_robot
            robot = _get_robot()
        robot._recover_held_state()
        if robot.held_object is not None:
            raise RuntimeError("RECOVERY_REQUIRED: gripper already holds an object")
        index = 0
        while index < len(steps):
            step = steps[index]
            if step["skill"] == "home":
                if robot.home() is not True:
                    raise RuntimeError("Home failed")
                emit({"skill": "home", "status": "SUCCESS"})
                index += 1
            else:
                transfer(robot, step["object"], steps[index+1]["zone"], emit)
                index += 2
        return {"status": "SUCCESS", "steps": records}
    except Exception as exc:
        status = exc.status if isinstance(exc, PlanError) else "FAILED"
        if "PERCEPTION_" in str(exc):
            status = "PERCEPTION_FAILED"
        elif "Cartesian path" in str(exc) or "IK branch" in str(exc) or "PLANNING_FAILED" in str(exc):
            status = "PLANNING_FAILED"
        return {"status": status, "message": str(exc), "steps": records}
