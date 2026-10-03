import os
import argparse
import shlex
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


def package_available(environment):
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "ur_simulation_gz"],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        return False
    return result.returncode == 0


def setup_candidates():
    script_dir = Path(__file__).resolve().parent
    configured = os.environ.get("UR3_GZ_SETUP")
    candidates = [
        Path(configured).expanduser() if configured else None,
        script_dir.parent / "install" / "setup.bash",
        script_dir.parent / "ur3_gz" / "install" / "setup.bash",
    ]
    return [path for path in candidates if path and path.is_file()]


def source_environment(setup_file):
    command = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(setup_file))} && env -0"
    )
    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
    )
    environment = {}
    for item in result.stdout.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            environment[os.fsdecode(key)] = os.fsdecode(value)
    return environment


def ros_environment():
    environment = dict(os.environ)
    if package_available(environment):
        return environment
    for setup_file in setup_candidates():
        environment = source_environment(setup_file)
        if package_available(environment):
            print(f"Using ROS workspace: {setup_file}", flush=True)
            return environment
    raise RuntimeError(
        "Package ur_simulation_gz is unavailable. Build the workspace with "
        "'colcon build --symlink-install', or set UR3_GZ_SETUP to its "
        "install/setup.bash file."
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
                    "green_cube",
                    "purple_cube",
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
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--command")
    mode.add_argument("--observe", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--scenario", choices=("1", "2"), default="1")
    args = parser.parse_args()
    simulation = demo = None

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        environment = ros_environment()
        if os.environ.get("HRI2_ENV_READY") != "1":
            environment["HRI2_ENV_READY"] = "1"
            os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], environment)
        os.environ.update(environment)
        simulation = subprocess.Popen(
            [
                "ros2",
                "launch",
                str(Path(__file__).with_name("launch") / "camera_moveit.launch.py"),
                "world_file:=" + str(Path(__file__).with_name("worlds") / (
                    "camera_table.sdf" if args.scenario == "1" else "demo2.sdf")),
                *(["gazebo_gui:=false", "moveit_rviz:=false"] if args.headless else []),
            ],
            env=environment,
            start_new_session=True,
        )
        wait_ready(simulation)
        demo = subprocess.Popen(
            [sys.executable, str(Path(__file__).with_name("main.py")),
             *(["--command", args.command] if args.command else ["--observe"] if args.observe else ["--demo"])],
            env=environment,
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
