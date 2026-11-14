"""Read-only inventory and opt-in, bounded checks for the M3 candidate stack."""

import argparse
import json
import os
import platform
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path

PX4_REVISION = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"  # v1.17.0 candidate
ROS_INTERFACES = (
    "rosgraph_msgs/msg/Clock",
    "px4_msgs/msg/SensorGps",
    "px4_msgs/msg/SensorCombined",
    "px4_msgs/msg/VehicleOdometry",
)


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str
    command: tuple[str, ...] = ()


def stop_group(process: subprocess.Popen[bytes], *, grace: int = 1) -> bool:
    deadline = time.monotonic() + grace
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait(timeout=grace)
        return False
    try:
        process.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        pass
    # The leader may exit before Gazebo or another child has drained.
    while time.monotonic() < deadline:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return False
        time.sleep(0.05)
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return False
    process.wait(timeout=grace)
    return True


def probe(
    name: str,
    *command: str,
    timeout: int = 5,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    output_limit: int = 16_384,
) -> Check:
    executable = shutil.which(command[0])
    if executable is None:
        return Check(name, "BLOCKED", f"Executable unavailable: {command[0]}", command)
    # Preflight runs on POSIX profiles. A private process group permits cleanup
    # of simulator children. Use a temporary file to avoid inherited pipe hangs.
    try:
        with tempfile.TemporaryFile() as log:
            with subprocess.Popen(
                (executable, *command[1:]),
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                cwd=cwd,
                env=env,
            ) as process:
                try:
                    code = process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    code = None
                finally:
                    stop_group(process)
            log.seek(0, os.SEEK_END)
            truncated = log.tell() > output_limit
            log.seek(max(0, log.tell() - output_limit))
            output = log.read().decode(errors="replace").strip()
    except (OSError, subprocess.TimeoutExpired) as error:
        return Check(name, "FAIL", str(error), command)
    if code is None:
        detail = f"Timed out after {timeout}s"
        return Check(name, "FAIL", f"{detail}: {output}" if output else detail, command)
    if truncated:
        return Check(
            name, "FAIL", f"Command output exceeded {output_limit} bytes; tail: " + output, command
        )
    if code:
        return Check(name, "FAIL", f"Exit {code}" + (f": {output}" if output else ""), command)
    return Check(name, "PASS", output or "Exited successfully", command)


def os_release() -> tuple[str, str]:
    try:
        lines = Path("/etc/os-release").read_text().splitlines()
    except OSError:
        return "", ""
    fields = {}
    for line in lines:
        key, separator, value = line.partition("=")
        if separator and key in {"ID", "VERSION_ID"}:
            try:
                words = shlex.split(value, comments=True)
            except ValueError:
                continue
            if len(words) == 1:
                fields[key] = words[0]
    return fields.get("ID", ""), fields.get("VERSION_ID", "")


def profile_checks(profile: str) -> list[Check]:
    system, machine = platform.system(), platform.machine().lower()
    linux_supported = system == "Linux" and machine in {"x86_64", "aarch64", "arm64"}
    apple_supported = system == "Darwin" and machine in {"arm64", "x86_64"}
    release = " ".join(os_release()) if system == "Linux" else platform.mac_ver()[0]
    host = f"{system} {machine} {release}".strip()
    if profile == "macos-dev" and apple_supported:
        return [Check("platform", "PASS", host)]
    if profile != "linux-integration" or not linux_supported or release != "ubuntu 24.04":
        return [Check("platform", "BLOCKED", f"{host} is outside profile {profile}")]
    checks = [Check("platform", "PASS", host)]
    if machine != "x86_64":
        checks.append(Check("architecture", "WARN", "Harmonic ARM64 is best effort"))
    return checks


def gaz_sim_version(check: Check) -> Check:
    if check.status != "PASS":
        return check
    match = re.fullmatch(
        r"(?:Gazebo Sim(?:ulator)?(?:,? version)?\s+)?(\d+)\.\d+\.\d+[^\s]*",
        check.detail.partition("\n")[0],
        flags=re.IGNORECASE,
    )
    if not match:
        return Check(
            check.name, "FAIL", f"Unrecognized gz-sim version: {check.detail}", check.command
        )
    if match.group(1) != "8":
        return Check(
            check.name, "BLOCKED", f"Expected Harmonic (gz-sim 8): {check.detail}", check.command
        )
    return check


def px4_checks(checkout: Path | None) -> list[Check]:
    if checkout is None:
        return [Check("PX4 SITL", "SKIP", "Supply --px4-checkout to inspect a candidate build")]
    checkout = checkout.resolve()
    top_level = probe(
        "PX4 checkout root", "git", "-C", str(checkout), "rev-parse", "--show-toplevel"
    )
    if top_level.status != "PASS" or Path(top_level.detail).resolve() != checkout:
        return [Check(top_level.name, "BLOCKED", f"Expected a PX4 repository root at {checkout}")]
    revision = probe("PX4 revision", "git", "-C", str(checkout), "rev-parse", "HEAD")
    if revision.status != "PASS" or revision.detail != PX4_REVISION:
        return [
            revision
            if revision.status != "PASS"
            else Check(
                "PX4 revision",
                "BLOCKED",
                f"Expected v1.17.0 ({PX4_REVISION}), got {revision.detail}",
            )
        ]
    source = probe(
        "PX4 source status",
        "git",
        "--no-optional-locks",
        "-C",
        str(checkout),
        "status",
        "--porcelain",
        "--untracked-files=no",
        "--ignore-submodules=none",
    )
    if source.status != "PASS" or source.detail != "Exited successfully":
        if source.status == "PASS":
            source = Check(source.name, "BLOCKED", "Tracked PX4 files or submodules have changed")
        return [revision, source]
    submodules = probe(
        "PX4 submodules",
        "git",
        "-C",
        str(checkout),
        "submodule",
        "status",
        "--recursive",
        timeout=20,
    )
    # git prefixes lines with '-' for missing, '+' for divergent, and 'U' for conflicts.
    if submodules.status != "PASS" or re.search(r"^[+U-]", submodules.detail, re.MULTILINE):
        if submodules.status == "PASS":
            submodules = Check(submodules.name, "BLOCKED", "Initialize the pinned PX4 submodules")
        return [revision, source, submodules]
    binary = checkout / "build/px4_sitl_default/bin/px4"
    return [revision, source, submodules, probe("PX4 SITL binary help", str(binary), "-h")]


def linux_checks(smoke: bool, checkout: Path | None, discover_topic: str | None) -> list[Check]:
    gz = gaz_sim_version(probe("Gazebo Harmonic", "gz", "sim", "--version"))
    ros = probe("ROS 2 CLI", "ros2", "--help")
    distro = os.environ.get("ROS_DISTRO", "")
    ros_distro = Check(
        "ROS distribution",
        "PASS" if distro == "jazzy" else "BLOCKED",
        distro or "Source the Jazzy underlay",
    )
    agent = shutil.which("MicroXRCEAgent") or shutil.which("micro-xrce-dds-agent")
    if agent:
        agent_check = Check(
            "Micro XRCE-DDS Agent", "WARN", f"Found {agent}; verify version 2.4.3 in the manifest"
        )
    else:
        agent_check = Check("Micro XRCE-DDS Agent", "BLOCKED", "Agent executable unavailable")
    checks = [gz, ros, ros_distro, agent_check]
    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_AUTOMATIC_DISCOVERY_RANGE"):
        checks.append(Check(key, "INFO", os.environ.get(key) or "Unset"))
    if not smoke:
        return checks + [Check("runtime probes", "SKIP", "Use --smoke to enable runtime probes")]

    if gz.status == "PASS":
        with tempfile.TemporaryDirectory(prefix="aerotwin-m3-") as tmp:
            world = Path(tmp) / "preflight.sdf"
            world.write_text(
                '<sdf version="1.9"><world name="preflight">'
                '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
                "</world></sdf>\n"
            )
            server = probe(
                "Gazebo server (10 requested iterations)",
                "gz",
                "sim",
                "-v",
                "4",
                "-s",
                "-r",
                "--iterations",
                "10",
                str(world),
                timeout=30,
                cwd=world.parent,
                env={**os.environ, "GZ_PARTITION": f"aerotwin_{uuid.uuid4().hex}"},
            )
            # The Harmonic CLI can exit zero even when server startup fails.
            if server.status == "PASS" and (
                "World [preflight] initialized" not in server.detail
                or "Shutting down gz-sim-server" not in server.detail
                or re.search(r"\[Err\]", server.detail)
            ):
                server = Check(
                    server.name,
                    "FAIL",
                    "Missing clean world startup/shutdown evidence: " + server.detail,
                    server.command,
                )
            checks.append(server)
    else:
        checks.append(Check("Gazebo server", "SKIP", "Requires a working gz-sim 8 CLI"))

    checks.extend(px4_checks(checkout))
    if ros.status != "PASS" or ros_distro.status != "PASS":
        return checks + [
            Check("ROS interfaces and discovery", "SKIP", "Requires a sourced Jazzy CLI")
        ]

    interfaces = [
        probe(interface, "ros2", "interface", "show", interface) for interface in ROS_INTERFACES
    ]
    bridge = probe("ros_gz_bridge", "ros2", "pkg", "prefix", "ros_gz_bridge")
    checks.extend([*interfaces, bridge])
    if (
        discover_topic
        and bridge.status == "PASS"
        and all(check.status == "PASS" for check in interfaces)
    ):
        discovery = probe(
            "DDS sample",
            "ros2",
            "topic",
            "echo",
            discover_topic,
            "px4_msgs/msg/SensorCombined",
            "--once",
            "--no-daemon",
            "--qos-profile",
            "sensor_data",
            timeout=15,
        )
        if discovery.status == "PASS" and not re.search(
            r"^timestamp: \d+$", discovery.detail, re.MULTILINE
        ):
            discovery = Check(
                discovery.name, "FAIL", "No SensorCombined timestamp in sample", discovery.command
            )
        checks.append(discovery)
    elif discover_topic:
        checks.append(Check("DDS sample", "BLOCKED", "Requires the ROS interfaces and bridge"))
    else:
        checks.append(
            Check(
                "DDS sample",
                "SKIP",
                "Requires interfaces and an explicit existing --discover-topic",
            )
        )
    return checks


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, choices=["linux-integration", "macos-dev"])
    parser.add_argument("--smoke", action="store_true", help="Enable opt-in Linux runtime checks")
    parser.add_argument(
        "--px4-checkout", type=Path, help="Existing v1.17.0 checkout and SITL build"
    )
    parser.add_argument("--discover-topic", help="Existing absolute SensorCombined ROS topic")
    parser.add_argument("--json", action="store_true", help="Emit structured JSON")
    args = parser.parse_args(argv)
    if (args.smoke and args.profile != "linux-integration") or (
        not args.smoke and (args.px4_checkout or args.discover_topic)
    ):
        parser.error("Use --profile linux-integration --smoke for runtime checks")
    if args.discover_topic and not re.fullmatch(r"/[A-Za-z_][\w/]*", args.discover_topic, re.ASCII):
        parser.error("--discover-topic requires an absolute ROS topic name")

    checks = profile_checks(args.profile)
    if checks[0].status == "PASS":
        checks.extend(probe(tool, tool, "--version") for tool in ("cmake", "c++", "python3", "git"))
        if args.profile == "linux-integration":
            checks.extend(linux_checks(args.smoke, args.px4_checkout, args.discover_topic))
        else:
            for tool in ("docker", "ros2", "gz", "px4", "MicroXRCEAgent"):
                checks.append(Check(tool, "INFO", shutil.which(tool) or "Not on PATH"))
            checks.append(Check("robotics runtime", "SKIP", "Use a validated Ubuntu environment"))
    elif args.smoke:
        checks.append(Check("runtime probes", "SKIP", "Platform prerequisite failed"))

    status = (
        1
        if any(check.status == "FAIL" for check in checks)
        else (2 if any(check.status == "BLOCKED" for check in checks) else 0)
    )
    if args.json:
        print(
            json.dumps(
                {
                    "profile": args.profile,
                    "exit_code": status,
                    "checks": [asdict(c) for c in checks],
                },
                indent=2,
            )
        )
    else:
        print(f"Profile: {args.profile}")
        for check in checks:
            print(f"{check.status:7} {check.name}: {check.detail}")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
