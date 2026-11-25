"""Container roles for the opt-in M3 transport experiment."""

import argparse
import json
import os
import signal
import socket
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path

from .m3_odometry_probe import odometry_sample, qos_allows_observation
from .m3_preflight import probe
from .m3_runtime import (
    REVISIONS,
    file_hash,
    owned_process,
    package_inventory,
    stats,
    wait_for_progress,
)

PX4 = Path("/work/PX4-Autopilot")
BUILD = PX4 / "build/px4_sitl_default"
AGENT = Path("/work/dds/agent-build/MicroXRCEAgent")
MSGS = Path("/work/dds/ros-overlay/src/px4_msgs")
EVIDENCE = Path("/evidence")
ARTIFACT_HASHES = {
    "px4": "babe228be6179e8f7abf8af0912ee6b6fae701e9e18cbacfb4d092670ac74402",
    "agent": "4cee94fb122336a32c9479e4fe2656bee80d7f445abd08a6534be725167ebc0b",
}


def checked(*command, timeout=10):
    result = probe(command[0], *map(str, command), timeout=timeout, output_limit=262144)
    if result.status != "PASS":
        raise RuntimeError(result.detail)
    return result.detail


def write_json(name, value):
    path = EVIDENCE / name
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


def inventory():
    revisions = {
        name: checked("git", "-C", path, "rev-parse", "HEAD")
        for name, path in {"px4": PX4, "agent": "/work/dds/agent", "px4_msgs": MSGS}.items()
    }
    if revisions != REVISIONS:
        raise RuntimeError(f"Revision mismatch: {revisions}")
    hashes = {"px4": file_hash(BUILD / "bin/px4"), "agent": file_hash(AGENT)}
    if hashes != ARTIFACT_HASHES:
        raise RuntimeError(f"Artifact identity differs from the M3.2.2 build: {hashes}")
    for name in ("VehicleOdometry", "SensorGps", "SensorCombined"):
        firmware = (
            PX4 / "msg" / ("versioned" if name == "VehicleOdometry" else "") / (name + ".msg")
        )
        if firmware.read_bytes() != (MSGS / "msg" / (name + ".msg")).read_bytes():
            raise RuntimeError(f"Message definition mismatch: {name}")
        installed = MSGS.parents[1] / "install/px4_msgs/share/px4_msgs/msg" / (name + ".msg")

        def definition(path):
            return [
                line.partition("#")[0].strip()
                for line in path.read_text().splitlines()
                if line.partition("#")[0].strip()
            ]

        if definition(installed) != definition(firmware):
            raise RuntimeError(f"Installed message definition mismatch: {name}")
    for artifact in (
        BUILD / "src/modules/simulation/gz_plugins/libOpticalFlowSystem.so",
        PX4 / "Tools/simulation/gz/models/x500/model.sdf",
        Path("/work/dds/ros-overlay/install/setup.bash"),
    ):
        if not artifact.is_file():
            raise RuntimeError(f"Missing artifact: {artifact}")
    version = checked("gz", "sim", "--versions")
    if version != "8.15.0":
        raise RuntimeError(f"Unverified Gazebo version: {version}")
    return {
        "revisions": revisions,
        "agent_dependencies": {
            path.parent.name: checked("git", "-C", path.parent, "rev-parse", "HEAD")
            for path in sorted(Path("/work/dds/agent-build").glob("*/src/*/.git"))
        },
        "installed_stack_packages": package_inventory(),
        "sha256": hashes,
        "gazebo": version,
        "architecture": checked("uname", "-m"),
        "packages": checked(
            "dpkg-query",
            "-W",
            "ros-jazzy-ros-base",
            "ros-jazzy-rclpy",
            "ros-jazzy-rmw-fastrtps-cpp",
        ),
        "ros_distribution": checked(
            "bash", "-c", "source /opt/ros/jazzy/setup.bash; echo $ROS_DISTRO"
        ),
    }


def sim_environment():
    assets = PX4 / "Tools/simulation/gz"
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PX4_", "GZ_", "IGN_"))}
    env.update(
        HEADLESS="1",
        PX4_SIM_MODEL="gz_x500",
        PX4_GZ_STANDALONE="1",
        PX4_GZ_WORLD="default",
        PX4_GZ_MODELS=str(assets / "models"),
        GZ_SIM_RESOURCE_PATH=f"{assets / 'models'}:{assets / 'worlds'}",
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(BUILD / "src/modules/simulation/gz_plugins"),
        GZ_SIM_SERVER_CONFIG_PATH=str(PX4 / "src/modules/simulation/gz_bridge/server.config"),
        GZ_IP="127.0.0.1",
        GZ_PARTITION=os.environ["AEROTWIN_RUN"],
        PX4_UXRCE_DDS_PORT="18888",
        PX4_UXRCE_DDS_NS=os.environ["AEROTWIN_RUN"],
        PATH=f"{BUILD / 'bin'}:{os.environ['PATH']}",
    )
    return env


def clock_identity():
    return {
        "namespace": os.readlink("/proc/self/ns/time"),
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "offsets": Path("/proc/self/timens_offsets").read_text().split(),
    }


def snapshot():
    env = sim_environment()
    # PX4 CLI connects to instance 0 via its Unix socket in /tmp.
    return {
        "clock_identity": clock_identity(),
        "wall_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "gazebo_stats": stats("default", env),
        "gyro": checked(BUILD / "bin/px4-listener", "sensor_gyro", "-n", "1"),
        "accel": checked(BUILD / "bin/px4-listener", "sensor_accel", "-n", "1"),
        "odometry": checked(BUILD / "bin/px4-listener", "vehicle_odometry", "-n", "1"),
        "timesync": checked(BUILD / "bin/px4-listener", "timesync_status", "-n", "1"),
        "xrce": checked(BUILD / "bin/px4-uxrce_dds_client", "status"),
    }


def sim():
    env = sim_environment()
    directory = Path("/tmp/aerotwin-session")
    directory.mkdir()
    if os.environ.get("AEROTWIN_SOURCE_CAPTURE") == "1":
        import shutil

        shutil.copytree(BUILD / "etc", directory / "etc")
        logging = directory / "etc/logging"
        logging.mkdir(parents=True)
        (logging / "logger_topics.txt").write_text(
            "vehicle_odometry 0\ntimesync_status 0\nsensor_combined 0\n"
        )
    (directory / "gz_env.sh").symlink_to(BUILD / "rootfs/gz_env.sh")
    logs = {}
    try:
        with ExitStack() as stack:
            gazebo, _ = stack.enter_context(
                owned_process(
                    ("gz", "sim", "-r", "-s", str(PX4 / "Tools/simulation/gz/worlds/default.sdf")),
                    directory,
                    env,
                    logs,
                )
            )
            px4, _ = stack.enter_context(
                owned_process(
                    (str(BUILD / "bin/px4"), "-d", "-w", str(directory), str(BUILD / "etc")),
                    directory,
                    env,
                    logs,
                )
            )
            progress = wait_for_progress("default", env, px4)
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                version = probe("startup", str(BUILD / "bin/px4-ver"), "all", timeout=3)
                if version.status == "PASS" and REVISIONS["px4"] in version.detail:
                    status = probe(
                        "client", str(BUILD / "bin/px4-uxrce_dds_client"), "status", timeout=3
                    )
                    if "Running" in status.detail:
                        break
                time.sleep(0.5)
            else:
                raise RuntimeError("PX4 startup did not become ready")
            address = socket.gethostbyname("xrce-agent")
            # Bypass the base image entrypoint. PX4's numeric -h address is explicit.
            checked(BUILD / "bin/px4-uxrce_dds_client", "stop")
            checked(
                BUILD / "bin/px4-uxrce_dds_client",
                "start",
                "-t",
                "udp",
                "-h",
                address,
                "-p",
                "18888",
                "-n",
                os.environ["AEROTWIN_RUN"],
            )
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                status = checked(BUILD / "bin/px4-uxrce_dds_client", "status")
                if "Running, connected" in status:
                    break
                time.sleep(0.5)
            else:
                raise RuntimeError("XRCE session connection timeout")
            write_json(
                "sim-ready.json",
                {
                    "version": version.detail,
                    "agent_ipv4": address,
                    "xrce": status,
                    "stats": progress,
                },
            )
            while True:
                if px4.poll() is not None or gazebo.poll() is not None:
                    raise RuntimeError("Simulator process exited unexpectedly")
                time.sleep(0.5)
    finally:
        write_json("sim-processes.json", logs)
        # Preserve full ULogs before the container is removed.
        if (directory / "log").is_dir():
            import shutil

            shutil.copytree(directory / "log", EVIDENCE / "ulog", dirs_exist_ok=True)


def dds_profile(peer):
    # UDP keeps transport independent of each container's private /dev/shm.
    deadline = time.monotonic() + 20
    while True:
        try:
            address = socket.gethostbyname(peer)
            break
        except socket.gaierror:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.2)
    discovery_port = 7400 + 250 * int(os.environ["ROS_DOMAIN_ID"]) + 10
    profile = Path("/tmp/aerotwin-fastdds.xml")
    profile.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
 <transport_descriptors>
  <transport_descriptor><transport_id>aerotwin_udp</transport_id><type>UDPv4</type></transport_descriptor>
 </transport_descriptors>
 <participant profile_name="aerotwin" is_default_profile="true">
  <rtps><builtin><initialPeersList>
  <locator><udpv4><address>{address}</address><port>{discovery_port}</port></udpv4></locator>
  </initialPeersList></builtin><userTransports><transport_id>aerotwin_udp</transport_id></userTransports>
  <useBuiltinTransports>false</useBuiltinTransports></rtps>
 </participant>
</profiles>
""")
    role = "agent" if peer == "ros-observer" else "observer"
    (EVIDENCE / f"{role}-dds-profile.xml").write_bytes(profile.read_bytes())
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
    os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = str(profile)


def agent():
    dds_profile("ros-observer")
    env = {k: v for k, v in os.environ.items() if k != "LD_LIBRARY_PATH"}
    logs = {}
    try:
        with owned_process(
            (str(AGENT), "udp4", "-p", "18888", "-v", "4"), Path("/tmp"), env, logs
        ) as (process, _):
            write_json("agent-ready.json", {"pid": process.pid, "bind": "0.0.0.0:18888"})
            if process.wait() != 0:
                raise RuntimeError("Agent exited")
    finally:
        write_json("agent-processes.json", logs)


def observer():
    dds_profile("xrce-agent")
    import rclpy
    from px4_msgs.msg import VehicleOdometry
    from rclpy.qos import QoSProfile, qos_check_compatible

    rclpy.init()
    node = rclpy.create_node("aerotwin_clock_observer")
    topic = f"/{os.environ['AEROTWIN_RUN']}/fmu/out/vehicle_odometry"
    qos = QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
    deadline = time.monotonic() + 60
    try:
        while time.monotonic() < deadline:
            endpoints = node.get_publishers_info_by_topic(topic)
            if endpoints:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            raise RuntimeError("DDS discovery timeout")
        if len(endpoints) != 1 or endpoints[0].topic_type != "px4_msgs/msg/VehicleOdometry":
            raise RuntimeError("Expected one typed PX4 publisher")
        offered = endpoints[0].qos_profile
        compatibility, reason = qos_check_compatible(offered, qos)
        if not qos_allows_observation(compatibility.name):
            raise RuntimeError(f"Incompatible QoS: {reason}")
        gid = list(endpoints[0].endpoint_gid)
        write_json(
            "observer-ready.json",
            {
                "topic": topic,
                "type": endpoints[0].topic_type,
                "publisher_gid": gid,
                "qos_compatibility": compatibility.name,
                "qos_reason": reason,
                "offered": {
                    "reliability": offered.reliability.name,
                    "durability": offered.durability.name,
                    "depth": offered.depth,
                },
                "requested": {
                    "reliability": qos.reliability.name,
                    "durability": qos.durability.name,
                    "depth": qos.depth,
                },
                "clock_identity": clock_identity(),
                "ros_clock": {
                    "type": node.get_clock().clock_type.name,
                    "use_sim_time": node.get_parameter("use_sim_time").value,
                },
            },
        )
        with (EVIDENCE / "samples.jsonl").open("w", buffering=1) as output:

            def receive(message):
                value = odometry_sample(message)
                value.update(
                    receipt_wall_ns=time.time_ns(),
                    receipt_monotonic_ns=time.monotonic_ns(),
                    ros_clock_ns=node.get_clock().now().nanoseconds,
                    publisher_gid=gid,
                )
                output.write(json.dumps(value, allow_nan=False) + "\n")

            subscription = node.create_subscription(VehicleOdometry, topic, receive, qos)
            last_heartbeat = 0
            with (EVIDENCE / "observer-clock.jsonl").open("w", buffering=1) as heartbeat:
                while True:
                    if time.monotonic() - last_heartbeat >= 0.5:
                        heartbeat.write(
                            json.dumps(
                                {
                                    "wall_ns": time.time_ns(),
                                    "monotonic_ns": time.monotonic_ns(),
                                    "ros_clock_ns": node.get_clock().now().nanoseconds,
                                }
                            )
                            + "\n"
                        )
                        last_heartbeat = time.monotonic()
                    rclpy.spin_once(node, timeout_sec=0.2)
                    final = node.get_publishers_info_by_topic(topic)
                    if len(final) != 1 or list(final[0].endpoint_gid) != gid:
                        write_json(
                            "publisher-change.json",
                            {"wall_ns": time.time_ns(), "count": len(final)},
                        )
                    # Keep the subscription alive throughout the bounded host experiment.
                    assert subscription is not None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("inventory", "sim", "agent", "observer", "snapshot"))
    args = parser.parse_args()

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        result = globals()[args.role]()
        if result is not None:
            print(json.dumps(result, indent=2))
        return 0
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
