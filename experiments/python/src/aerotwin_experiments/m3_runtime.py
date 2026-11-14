"""Opt-in M3.2 runtime validation on an isolated Ubuntu 24.04 amd64 host."""

import argparse
import hashlib
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid
from contextlib import ExitStack, contextmanager
from dataclasses import asdict
from pathlib import Path

from . import m3_preflight as preflight
from .m3_preflight import Check, probe

REVISIONS = {
    "px4": preflight.PX4_REVISION,
    "agent": "73622810d984349b80bbac0ef55fc0b694d62222",
    "px4_msgs": "86d8239e962f6939e05c3737784f60c02fa884db",
}
PACKAGE_FILTERS = (
    "ros-jazzy-*",
    "gz-*",
    "libgz-*",
    "sdformat*",
    "libsdformat*",
)
PACKAGE_COMMAND = (
    "dpkg-query",
    "-W",
    "-f=${db:Status-Abbrev}\t${binary:Package}\t${Version}\n",
    *PACKAGE_FILTERS,
)
WORLD = (
    '<sdf version="1.9"><world name="preflight">'
    '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
    "</world></sdf>\n"
)
LIVE_GATES = ("Gazebo clock/pause/reset", "x500 run 1", "x500 run 2", "Cross-domain clock mapping")


def package_inventory() -> dict[str, str]:
    check = probe("Installed stack packages", *PACKAGE_COMMAND, timeout=10, output_limit=262144)
    # dpkg-query returns 1 for an unmatched pattern, but still prints matched packages.
    if check.status == "FAIL" and not check.detail.startswith("Exit 1:"):
        raise ValueError(check.detail)
    packages = {}
    for line in check.detail.splitlines():
        fields = line.removeprefix("Exit 1: ").split("\t")
        if len(fields) == 3 and fields[0].strip() == "ii":
            packages[fields[1]] = fields[2]
    if not packages:
        raise ValueError("No installed ROS or Gazebo packages were found")
    return packages


def file_hash(path: Path) -> str:
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def manifest_checks(manifest: dict, paths: dict[str, Path]) -> list[Check]:
    checks = []
    if manifest.get("revisions") != REVISIONS:
        checks.append(
            Check("Source manifest", "BLOCKED", "Source revisions differ from candidates")
        )
    for name in REVISIONS:
        root = paths[name]
        head = probe(name, "git", "-C", str(root), "rev-parse", "HEAD", "--show-toplevel")
        if head.status != "PASS" or head.detail.splitlines() != [REVISIONS[name], str(root)]:
            checks.append(Check(name, "BLOCKED", f"Wrong checkout or revision: {head.detail}"))
            continue
        status = probe(
            name,
            "git",
            "--no-optional-locks",
            "-C",
            str(root),
            "status",
            "--porcelain",
            "--untracked-files=no",
            "--ignore-submodules=none",
        )
        if status.status != "PASS" or status.detail != "Exited successfully":
            checks.append(Check(name, "BLOCKED", f"Checkout is not clean: {status.detail}"))
        else:
            checks.append(Check(name, "PASS", REVISIONS[name]))
    try:
        packages = package_inventory()
        if packages != manifest.get("packages"):
            checks.append(Check("Package manifest", "BLOCKED", "Installed package versions differ"))
        else:
            checks.append(Check("Package manifest", "PASS", f"{len(packages)} versions match"))
    except ValueError as error:
        checks.append(Check("Package manifest", "BLOCKED", str(error)))
    for name in ("px4_binary", "agent_binary"):
        if not os.access(paths[name], os.X_OK):
            checks.append(Check(name, "BLOCKED", "Executable missing or lacks execute permission"))
            continue
        try:
            digest = file_hash(paths[name])
        except OSError as error:
            checks.append(Check(name, "BLOCKED", str(error)))
        else:
            status = "PASS" if digest == manifest.get(name + "_sha256") else "BLOCKED"
            checks.append(Check(name, status, f"SHA-256 {digest}"))
    if (
        not isinstance(manifest.get("package_origins"), str)
        or not manifest["package_origins"].strip()
    ):
        checks.append(
            Check("Package origins", "BLOCKED", "Record package repositories in the manifest")
        )
    if manifest.get("agent_version") != "2.4.3":
        checks.append(Check("Agent build provenance", "BLOCKED", "Record the v2.4.3 build"))
    if not manifest.get("agent_dependencies"):
        checks.append(
            Check("Agent build dependencies", "BLOCKED", "Record Agent dependency revisions")
        )
    return checks


def inventory_checks(paths: dict[str, Path], env: dict[str, str]) -> list[Check]:
    checks = preflight.linux_checks(False, None, None)
    # The chosen Agent has an explicit path and a pinned source/build manifest.
    checks = [c for c in checks if c.name not in {"Micro XRCE-DDS Agent", "runtime probes"}]
    versions = probe("Installed Gazebo Sim versions", "gz", "sim", "--versions")
    if versions.status == "PASS" and not re.fullmatch(r"8\.\d+\.\d+[^\s]*", versions.detail):
        versions = Check(versions.name, "BLOCKED", f"Expected only gz-sim 8: {versions.detail}")
    checks.append(versions)
    checks.extend(preflight.px4_checks(paths["px4"]))
    checks.extend(
        probe(name, "ros2", "interface", "show", name, env=env) for name in preflight.ROS_INTERFACES
    )
    checks.append(probe("ROS RMW", "ros2", "pkg", "prefix", "rmw_fastrtps_cpp", env=env))
    checks.append(
        probe(
            "ROS probe Python ABI",
            sys.executable,
            "-c",
            "import rclpy; from px4_msgs.msg import VehicleOdometry; "
            "VehicleOdometry.__class__.__import_type_support__(); "
            "assert VehicleOdometry.__class__._TYPE_SUPPORT is not None",
            env=env,
        )
    )
    checks.append(probe("ROS Gazebo bridge", "ros2", "pkg", "prefix", "ros_gz_bridge", env=env))
    # Verify installed definitions from the actual sourced overlay.
    prefix = probe("ROS message overlay", "ros2", "pkg", "prefix", "px4_msgs", env=env)
    checks.append(prefix)
    if prefix.status == "PASS":
        for name in ("SensorGps", "SensorCombined", "VehicleOdometry"):
            try:
                msg = name + ".msg"
                installed = Path(prefix.detail) / "share/px4_msgs/msg" / msg
                source = paths["px4_msgs"] / "msg" / msg
                versioned = "versioned" if name == "VehicleOdometry" else ""
                firmware = paths["px4"] / "msg" / versioned / msg

                def definitions(path):
                    return [
                        line.partition("#")[0].strip()
                        for line in path.read_text().splitlines()
                        if line.partition("#")[0].strip()
                    ]

                if source.read_bytes() != firmware.read_bytes() or definitions(
                    installed
                ) != definitions(source):
                    raise ValueError(
                        f"{name} differs between firmware, source or installed overlay"
                    )
            except (OSError, ValueError) as error:
                checks.append(Check(f"{name} definition", "BLOCKED", str(error)))
            else:
                checks.append(
                    Check(f"{name} definition", "PASS", "Firmware, source and overlay match")
                )
    for port in {int(env["PX4_UXRCE_DDS_PORT"]), 14580, 14280, 13030, 18570}:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                udp.bind(("0.0.0.0", port))
        except OSError as error:
            checks.append(Check("Runtime UDP ports", "BLOCKED", f"{port}: {error}"))
    for path in (
        paths["px4_binary"].parent.parent / "rootfs/gz_env.sh",
        paths["px4_binary"].parent.parent / "etc/init.d-posix/rcS",
        paths["px4"] / "Tools/simulation/gz/models/x500/model.sdf",
        paths["px4"] / "Tools/simulation/gz/worlds/default.sdf",
    ):
        if not path.is_file():
            checks.append(Check("PX4 runtime asset", "BLOCKED", f"Missing {path}"))
    return checks


def log_tail(log) -> str:
    # pread keeps the child's shared descriptor offset untouched.
    size = os.fstat(log.fileno()).st_size
    return os.pread(log.fileno(), 8192, max(0, size - 8192)).decode(errors="replace")


@contextmanager
def owned_process(command: tuple[str, ...], cwd: Path, env: dict[str, str], logs: dict):
    with tempfile.TemporaryFile(dir=cwd) as log:
        binary = shutil.which(command[0], path=env.get("PATH"))
        if binary is None:
            raise FileNotFoundError(command[0])
        process = subprocess.Popen(
            (binary, *command[1:]),
            cwd=cwd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            yield process, log
        finally:
            try:
                forced = preflight.stop_group(process, grace=5)
            finally:
                logs[command[0]] = {
                    "command": command,
                    "tail": log_tail(log),
                    "exit_code": process.poll(),
                }
            logs[command[0]]["forced_cleanup"] = forced


def stats(world: str, env: dict[str, str]) -> list[tuple[int, bool]]:
    result = probe(
        "Gazebo statistics",
        "gz",
        "topic",
        "-e",
        "-t",
        f"/world/{world}/stats",
        "-n",
        "2",
        timeout=5,
        env=env,
    )
    if result.status != "PASS":
        raise RuntimeError(result.detail)
    samples = []
    for message in re.split(r"\n\s*\n", result.detail):
        match = re.search(r"\bsim_time\s*\{([^}]*)\}", message)
        if not match:
            continue
        sec = re.search(r"\bsec:\s*(-?\d+)\b", match[1])
        nsec = re.search(r"\bnsec:\s*(-?\d+)\b", match[1])
        nanos = int(nsec[1]) if nsec else 0
        if not 0 <= nanos < 1_000_000_000 or (sec and int(sec[1]) < 0):
            raise RuntimeError("Gazebo statistics contained an invalid nanosecond component")
        samples.append(
            (
                (int(sec[1]) if sec else 0) * 1_000_000_000 + nanos,
                bool(re.search(r"^paused:\s*true\b", message, re.MULTILINE)),
            )
        )
    if len(samples) != 2:
        raise RuntimeError(f"Expected two Gazebo statistics samples; got {result.detail}")
    return samples


def wait_for_progress(world: str, env: dict[str, str], process: subprocess.Popen) -> list:
    deadline = time.monotonic() + 30
    detail = "No progressing simulation"
    while time.monotonic() < deadline and process.poll() is None:
        try:
            samples = stats(world, env)
            if samples[1][0] > samples[0][0] and not any(paused for _, paused in samples):
                return samples
        except RuntimeError as error:
            detail = str(error)
        time.sleep(0.1)
    raise RuntimeError(f"Gazebo progress check failed: {detail}; process exit={process.poll()}")


def control(request: str, env: dict[str, str]) -> None:
    check = probe(
        "Gazebo world control",
        "gz",
        "service",
        "-s",
        "/world/preflight/control",
        "--reqtype",
        "gz.msgs.WorldControl",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        request,
        env=env,
    )
    if check.status != "PASS" or not re.search(r"^data:\s*true\s*$", check.detail, re.MULTILINE):
        raise RuntimeError(f"Gazebo rejected {request}: {check.detail}")


def gazebo_clock(directory: Path, env: dict[str, str], evidence: dict) -> None:
    world = directory / "preflight.sdf"
    world.write_text(WORLD)
    with owned_process(
        ("gz", "sim", "-s", "-r", "-v", "4", str(world)),
        directory,
        env,
        evidence.setdefault("logs", {}),
    ) as (process, _):
        evidence["running"] = wait_for_progress("preflight", env, process)
        control("pause: true", env)
        paused = stats("preflight", env)
        evidence["paused"] = paused
        if not all(flag for _, flag in paused) or paused[0][0] != paused[1][0]:
            raise RuntimeError("Gazebo did not hold simulation time while paused")
        control("reset: {time_only: true}", env)
        reset = stats("preflight", env)
        evidence["reset"] = reset
        if (
            not all(flag for _, flag in reset)
            or reset[0][0] != reset[1][0]
            or reset[1][0] >= paused[1][0]
        ):
            raise RuntimeError("No backward simulation-time jump observed after time reset")
        control("pause: false", env)
        evidence["resumed"] = wait_for_progress("preflight", env, process)


def sitl_run(
    directory: Path, env: dict[str, str], paths: dict[str, Path], topic: str, evidence: dict
) -> None:
    build = paths["px4_binary"].parent.parent
    (directory / "gz_env.sh").symlink_to(build / "rootfs/gz_env.sh")
    command = (str(paths["px4_binary"]), "-d", "-w", str(directory), str(build / "etc"))
    agent = (str(paths["agent_binary"]), "udp4", "-p", env["PX4_UXRCE_DDS_PORT"], "-v", "4")
    start = time.monotonic()
    with ExitStack() as stack:
        logs = evidence.setdefault("logs", {})
        agent_process, _ = stack.enter_context(owned_process(agent, directory, env, logs))
        px4, _ = stack.enter_context(owned_process(command, directory, env, logs))
        evidence["gazebo_stats"] = wait_for_progress("default", env, px4)
        evidence["gazebo_ready_seconds"] = round(time.monotonic() - start, 3)
        check = probe(
            "ROS 2 decoded PX4 odometry",
            sys.executable,
            "-m",
            "aerotwin_experiments.m3_odometry_probe",
            topic,
            timeout=25,
            env=env,
        )
        evidence["ros_command"] = check.command
        for line in reversed(check.detail.splitlines()):
            try:
                observation = json.loads(line.removeprefix("Exit 1: "))
            except json.JSONDecodeError:
                continue
            if isinstance(observation, dict) and "semantics_passed" in observation:
                evidence["ros"] = observation
                break
        else:
            evidence["ros_output"] = check.detail
        evidence["observation_seconds"] = round(time.monotonic() - start, 3)
        evidence["xrce_exchange"] = "NOT_RUN"
        if (
            check.status != "PASS"
            or not evidence.get("ros", {}).get("semantics_passed")
            or px4.poll() is not None
            or agent_process.poll() is not None
        ):
            evidence["xrce_exchange"] = "FAIL"
            raise RuntimeError("No validated odometry exchange while PX4 and Agent were running")
        evidence["xrce_exchange"] = "PASS"


def run_gate(name, action, *args) -> dict:
    evidence: dict = {}
    try:
        action(*args, evidence)
        if any(
            log["forced_cleanup"] or log["exit_code"] not in (0, -signal.SIGTERM)
            for log in evidence.get("logs", {}).values()
        ):
            raise RuntimeError("A process required forced cleanup or exited unexpectedly")
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        return {"name": name, "status": "FAIL", "detail": str(error), "evidence": evidence}
    return {"name": name, "status": "PASS", "evidence": evidence}


def live_report(paths: dict[str, Path], manifest: dict, domain: int, port: int) -> list[dict]:
    # Remove caller overrides that can point PX4 at another simulator or agent.
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PX4_", "GZ_", "IGN_"))}
    for key in (
        "CYCLONEDDS_URI",
        "FASTDDS_DEFAULT_PROFILES_FILE",
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "ROS_LOCALHOST_ONLY",
        "ROS_DISCOVERY_SERVER",
        "RMW_FASTRTPS_USE_QOS_FROM_XML",
    ):
        env.pop(key, None)
    namespace = f"aerotwin_{uuid.uuid4().hex}"
    env.update(
        HEADLESS="1",
        PX4_SIM_MODEL="gz_x500",
        GZ_IP="127.0.0.1",
        GZ_PARTITION=namespace,
        ROS_DOMAIN_ID=str(domain),
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
        ROS_STATIC_PEERS="",
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
        PX4_UXRCE_DDS_NS=namespace,
        PX4_UXRCE_DDS_PORT=str(port),
    )
    checks = manifest_checks(manifest, paths)
    if all(check.status == "PASS" for check in checks):
        checks.extend(inventory_checks(paths, env))
    results = [asdict(c) for c in checks]
    if not any(check.status in {"FAIL", "BLOCKED"} for check in checks):
        results.append({"name": "Verified manifest", "status": "PASS", "values": manifest})
    results.append(
        {
            "name": "Run environment",
            "status": "PASS",
            "values": {
                key: env.get(key)
                for key in (
                    "HEADLESS",
                    "PX4_SIM_MODEL",
                    "GZ_IP",
                    "GZ_PARTITION",
                    "ROS_DOMAIN_ID",
                    "ROS_AUTOMATIC_DISCOVERY_RANGE",
                    "RMW_IMPLEMENTATION",
                    "PX4_UXRCE_DDS_NS",
                    "PX4_UXRCE_DDS_PORT",
                )
            },
        }
    )
    if any(check.status in {"FAIL", "BLOCKED"} for check in checks):
        return results + [
            {"name": gate, "status": "BLOCKED", "detail": "Prerequisite failed"}
            for gate in LIVE_GATES
        ]
    with tempfile.TemporaryDirectory(prefix="aerotwin-m32-") as directory:
        root = Path(directory)
        home = root / "home"
        home.mkdir()
        env = {
            **env,
            "HOME": str(home),
            "XDG_CONFIG_HOME": str(home / "config"),
            "XDG_CACHE_HOME": str(home / "cache"),
            "XDG_DATA_HOME": str(home / "data"),
        }
        gazebo = run_gate(LIVE_GATES[0], gazebo_clock, root, env)
        results.append(gazebo)
        for number in (1, 2):
            if gazebo["status"] != "PASS":
                results.append(
                    {
                        "name": LIVE_GATES[number],
                        "status": "BLOCKED",
                        "detail": "Gazebo clock probe failed",
                    }
                )
                continue
            work = root / str(number)
            work.mkdir()
            # Each repetition gets a distinct topic namespace and Gazebo partition.
            identity = f"{namespace}_{number}"
            run_env = {**env, "GZ_PARTITION": identity, "PX4_UXRCE_DDS_NS": identity}
            topic = f"/{identity}/fmu/out/vehicle_odometry"
            results.append(run_gate(LIVE_GATES[number], sitl_run, work, run_env, paths, topic))
    return results + [
        {
            "name": LIVE_GATES[3],
            "status": "NOT_RUN",
            "detail": "Separate ROS /clock and PX4 clock-domain synchronization experiment required",
        }
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("plan", "inventory", "live"))
    for name in ("manifest", "px4", "agent", "px4-msgs", "agent-binary"):
        parser.add_argument(f"--{name}", type=Path)
    parser.add_argument("--domain", type=int)
    parser.add_argument("--port", type=int)
    parser.add_argument(
        "--isolated-host",
        action="store_true",
        help="Confirm a dedicated test host/network with no physical vehicles or other SITL runs",
    )
    args = parser.parse_args(argv)
    if args.operation == "plan":
        print(
            json.dumps(
                {
                    "reference": "Ubuntu 24.04 amd64 / Jazzy / Harmonic",
                    "revisions": REVISIONS,
                    "agent_version": "2.4.3",
                },
                indent=2,
            )
        )
        return 0

    checks = preflight.profile_checks("linux-integration")
    if any(c.status != "PASS" for c in checks):
        results = [asdict(c) for c in checks[:1]]
        results.append(
            {
                "name": "Reference platform",
                "status": "BLOCKED",
                "detail": "Ubuntu 24.04 amd64 required",
            }
        )
    elif args.operation == "inventory":
        try:
            print(
                json.dumps(
                    {
                        "revisions": REVISIONS,
                        "packages": package_inventory(),
                        "agent_version": "2.4.3",
                    },
                    indent=2,
                )
            )
            return 0
        except ValueError as error:
            results = [{"name": "Package inventory", "status": "BLOCKED", "detail": str(error)}]
    elif not args.isolated_host:
        results = [
            {
                "name": "Host isolation",
                "status": "BLOCKED",
                "detail": "Confirm the documented isolation prerequisites with --isolated-host",
            }
        ]
    else:
        if not all((args.manifest, args.px4, args.agent, args.px4_msgs, args.agent_binary)):
            parser.error("live requires --manifest, --px4, --agent, --px4-msgs and --agent-binary")
        if args.domain not in range(1, 102) or args.port not in range(1024, 65536):
            parser.error("live requires explicit --domain (1–101) and --port (1024–65535)")
        paths = {
            name: getattr(args, name).resolve()
            for name in ("px4", "agent", "px4_msgs", "agent_binary")
        }
        paths["px4_binary"] = paths["px4"] / "build/px4_sitl_default/bin/px4"
        try:
            manifest = json.loads(args.manifest.read_text())
            if not isinstance(manifest, dict):
                raise ValueError("Expected a JSON object")

            def interrupt(signum, _frame):
                signal.signal(signum, signal.SIG_IGN)
                raise KeyboardInterrupt

            previous = signal.signal(signal.SIGTERM, interrupt)
            try:
                results = live_report(paths, manifest, args.domain, args.port)
            finally:
                signal.signal(signal.SIGTERM, previous)
        except (OSError, ValueError) as error:
            results = [{"name": "Runtime inputs", "status": "BLOCKED", "detail": str(error)}]
        except KeyboardInterrupt:
            results = [
                {
                    "name": "Run interrupted",
                    "status": "FAIL",
                    "detail": "Process group cleanup requested",
                }
            ]

    present = {result["name"] for result in results}
    results.extend(
        {"name": gate, "status": "BLOCKED", "detail": "Required setup unavailable"}
        for gate in LIVE_GATES
        if gate not in present
    )
    statuses = {result["status"] for result in results}
    code = 1 if "FAIL" in statuses else 2 if "BLOCKED" in statuses or "NOT_RUN" in statuses else 0
    print(json.dumps({"exit_code": code, "checks": results}, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
