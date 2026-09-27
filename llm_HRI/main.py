import argparse
import json
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor

from llm import PlanError, parse_command
from skill_executor import execute_plan


DEMO_TRANSFERS = (
    ("red_cube", "zone_a"),
    ("blue_cube", "zone_b"),
    ("yellow_cube", "zone_c"),
)


def build_demo_plan():
    steps = [{"skill": "home"}]
    for object_name, zone_name in DEMO_TRANSFERS:
        steps.extend(
            [
                {"skill": "pick", "object": object_name},
                {"skill": "place", "object": object_name, "zone": zone_name},
            ]
        )
    steps.append({"skill": "home"})
    return {"plan": steps}


def process_command(command, dry_run=False, emit=None):
    emit = emit or (lambda event: None)
    try:
        plan = parse_command(command)
    except PlanError as exc:
        return {"status": exc.status, "message": str(exc), "steps": []}
    emit({"event": "plan", **plan})
    if dry_run:
        return {"status": "PLAN_READY", **plan, "executed": False}
    return execute_plan(plan, on_step=lambda step: emit({"event": "step", **step}))


def run_node(dry_run, ros_args):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    class CommandNode(Node):
        def __init__(self):
            super().__init__("hri_language_commands")
            self.output = self.create_publisher(String, "/hri/result", 10)
            self.create_subscription(String, "/hri/command", self.receive, 10)
            self.pool = ThreadPoolExecutor(max_workers=1)
            self.busy = threading.Lock()
            self.get_logger().info("Ready: /hri/command -> Gemini -> skills -> /hri/result")

        def publish(self, request_id, event):
            if self.context.ok():
                payload = {"request_id": request_id, **event}
                self.output.publish(
                    String(data=json.dumps(payload, ensure_ascii=False))
                )

        def receive(self, message):
            request_id = uuid.uuid4().hex
            if not self.busy.acquire(blocking=False):
                self.publish(request_id, {"event": "result", "status": "BUSY"})
                return
            self.publish(request_id, {"event": "accepted", "status": "RUNNING"})
            self.pool.submit(self.work, request_id, message.data)

        def work(self, request_id, command):
            try:
                result = process_command(
                    command,
                    dry_run,
                    lambda event: self.publish(request_id, event),
                )
                self.publish(request_id, {"event": "result", **result})
            except Exception:
                self.publish(
                    request_id,
                    {
                        "event": "result",
                        "status": "FAILED",
                        "message": "Command processing failed",
                    },
                )
            finally:
                if not dry_run:
                    from action import shutdown

                    shutdown()
                self.busy.release()

    rclpy.init(args=ros_args)
    node = CommandNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=True)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--command", help="Run one English instruction")
    mode.add_argument(
        "--demo",
        action="store_true",
        help="Red -> A, Blue -> B, Yellow -> C, without Gemini",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Parse/validate only; no motion"
    )
    args, ros_args = parser.parse_known_args()
    if args.command is None and not args.demo:
        run_node(args.dry_run, ros_args)
        return

    def emit(event):
        print(json.dumps(event, ensure_ascii=False), flush=True)

    try:
        if args.demo:
            plan = build_demo_plan()
            if args.dry_run:
                result = {"status": "PLAN_READY", **plan, "executed": False}
            else:
                result = execute_plan(
                    plan,
                    on_step=lambda step: emit({"event": "step", **step}),
                )
        else:
            result = process_command(args.command, args.dry_run, emit)
        emit({"event": "result", **result})
    finally:
        if not args.dry_run:
            from action import shutdown

            shutdown()
    if result["status"] not in {"SUCCESS", "PLAN_READY"}:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
