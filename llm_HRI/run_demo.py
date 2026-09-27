import os
import signal
import subprocess
import sys
import time
from pathlib import Path


SHUTDOWN_STAGES = (
    (signal.SIGINT, 12),
    (signal.SIGTERM, 5),
    (signal.SIGKILL, 2),
)


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            pass
    for sig, timeout in SHUTDOWN_STAGES:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.1)
    process.wait(timeout=5)


def wait_ready(simulation):
    import rclpy
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene

    rclpy.init()
    node = rclpy.create_node("demo_readiness")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    deadline = time.monotonic() + 120
    try:
        while time.monotonic() < deadline:
            if simulation.poll() is not None:
                raise RuntimeError("Simulation exited; close any existing launch first")
            if not client.wait_for_service(timeout_sec=1):
                continue
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if future.done() and future.result() is not None:
                names = {
                    obj.id for obj in future.result().scene.world.collision_objects
                }
                required = {
                    "work_table",
                    "red_cube",
                    "blue_cube",
                    "yellow_cube",
                    "zone_a",
                    "zone_b",
                    "zone_c",
                }
                if required <= names:
                    return
            time.sleep(0.2)
        raise RuntimeError("MoveIt scene not ready within 120 seconds")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    simulation = demo = None

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        simulation = subprocess.Popen(
            [
                "ros2",
                "launch",
                "ur_simulation_gz",
                "pick_place_moveit.launch.py",
            ],
            start_new_session=True,
        )
        wait_ready(simulation)
        demo = subprocess.Popen(
            [sys.executable, str(Path(__file__).with_name("main.py")), "--demo"],
            start_new_session=True,
        )
        while demo.poll() is None:
            if simulation.poll() is not None:
                raise RuntimeError("Simulation closed before demo finished")
            time.sleep(0.2)
        return demo.returncode
    except KeyboardInterrupt:
        return 130
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for process in (demo, simulation):
            if process is not None:
                stop(process)
        print("Demo stopped; owned processes cleaned up.", flush=True)


if __name__ == "__main__":
    sys.exit(main())
