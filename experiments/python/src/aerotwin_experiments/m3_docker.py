"""Opt-in snapshot-based multi-container M3 DDS and clock experiment."""

import argparse
import json
import os
import re
import signal
import time
import uuid
from pathlib import Path

from .m3_odometry_probe import sample_series_valid
from .m3_preflight import probe

SERVICES = ("xrce-agent", "px4-sim", "ros-observer")


def command(*args, env=None, timeout=15):
    result = probe(str(args[0]), *map(str, args), env=env, timeout=timeout, output_limit=2_000_000)
    if result.status != "PASS":
        raise RuntimeError(f"{' '.join(map(str, args))}: {result.detail}")
    return "" if result.detail == "Exited successfully" else result.detail


def save(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


def samples(path):
    if not path.exists():
        return []
    lines = path.read_text().splitlines()
    values = []
    for index, line in enumerate(lines):
        try:
            values.append(json.loads(line))
        except json.JSONDecodeError:
            if index != len(lines) - 1:
                raise
    return values


def ordered_window(values):
    """Find three consecutive valid samples with stable source reset metadata."""
    return next(
        (
            values[i : i + 3]
            for i in range(len(values) - 2)
            if sample_series_valid(values[i : i + 3])
        ),
        [],
    )


def timestamp_changes(values):
    return {
        "count": len(values),
        "backward_publication": sum(
            b["timestamp_us"] < a["timestamp_us"] for a, b in zip(values, values[1:])
        ),
        "duplicate_publication": sum(
            b["timestamp_us"] == a["timestamp_us"] for a, b in zip(values, values[1:])
        ),
        "backward_acquisition": sum(
            b["timestamp_sample_us"] < a["timestamp_sample_us"] for a, b in zip(values, values[1:])
        ),
        "invalid": sum(not sample["valid"] for sample in values),
    }


def clock_status(phase, before_reset=None):
    """Classify observations without equating Gazebo rollback with a PX4 reset."""
    first, last = phase["start"], phase["end"]
    initial, final = first["gazebo_stats"], last["gazebo_stats"]
    progressing = final[-1][0] > initial[0][0] and not any(flag for _, flag in initial + final)
    if before_reset is not None:
        return (
            "PASS"
            if progressing and initial[0][0] < before_reset["gazebo_stats"][-1][0]
            else "FAIL"
        )
    if first["phase"].startswith("pause"):
        held = len({value for value, flag in initial + final if flag}) == 1
        return "PASS" if held and all(flag for _, flag in initial + final) else "FAIL"
    messages = phase["messages"]
    ordered = not any(
        messages.get(name, 0)
        for name in (
            "backward_publication",
            "duplicate_publication",
            "backward_acquisition",
            "invalid",
        )
    )
    return "PASS" if progressing and ordered and messages["count"] >= 3 else "FAIL"


def validate_cleanup(states, logs):
    for state in states:
        if state["Running"] or state["OOMKilled"] or state["ExitCode"] != 0:
            raise RuntimeError(f"Unclean service exit: {state}")
    for role in ("sim", "agent"):
        if not logs.get(role):
            raise RuntimeError(f"Missing {role} process cleanup evidence")
        for log in logs[role].values():
            if log["forced_cleanup"] or log["exit_code"] not in (0, -signal.SIGTERM):
                raise RuntimeError(f"Unclean child process: {log}")


def acceptance(reports):
    transport = len(reports) == 2 and all(
        r["transport"] == r["cleanup"] == "PASS" and r.get("removed") for r in reports
    )
    clocks = {}
    for name in ("normal", "pause", "resume", "time_only_reset"):
        statuses = [r.get("clocks", {}).get(name, {}).get("status", "NOT_RUN") for r in reports]
        clocks[name] = (
            "PASS"
            if len(statuses) == 2 and statuses == ["PASS", "PASS"]
            else ("FAIL" if "FAIL" in statuses else "NOT_RUN")
        )
    restarted = transport and (
        reports[0]["observer"]["publisher_gid"] != reports[1]["observer"]["publisher_gid"]
        and reports[0]["identity"] != reports[1]["identity"]
        and all(r["sim"]["stats"][0][0] < 1_000_000_000 for r in reports)
    )
    return {
        "multi_container_transport": "PASS" if transport else "FAIL",
        **clocks,
        "controlled_full_restart": "PASS" if restarted else "NOT_RUN",
        "coupled_in_place_reset": "BLOCKED",
        "clock_domain_mapping": "BLOCKED",
        "production_epoch_policy": "NOT_RUN",
        "m34_readiness": "BLOCKED",
        "ubuntu_amd64_reference": "NOT_RUN",
    }


class Session:
    def __init__(self, compose, image, directory):
        self.directory = directory
        self.identity = "aerotwin_m33_" + uuid.uuid4().hex[:12]
        self.env = dict(
            os.environ,
            AEROTWIN_RUNTIME_IMAGE=image,
            AEROTWIN_RUN=self.identity,
            AEROTWIN_EVIDENCE=str(directory),
            AEROTWIN_PYTHON_SOURCE=str(Path(__file__).resolve().parents[1]),
        )
        self.prefix = (
            "docker",
            "compose",
            "--progress",
            "quiet",
            "-p",
            self.identity,
            "-f",
            str(compose),
        )
        self.checkpoints = []

    def call(self, *args, timeout=30):
        return command(*self.prefix, *args, env=self.env, timeout=timeout)

    def execute(self, service, *args, timeout=20):
        return self.call("exec", "-T", service, *args, timeout=timeout)

    def inspect(self, service):
        container = self.call("ps", "-aq", service)
        if not container or "\n" in container:
            raise RuntimeError(f"Expected one {service} container, got {container}")
        return json.loads(command("docker", "inspect", container))[0]

    def ready(self, filename, service, timeout=65):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.directory / filename).exists():
                return json.loads((self.directory / filename).read_text())
            state = self.inspect(service)["State"]
            if not state["Running"]:
                raise RuntimeError(f"{service} exited before readiness: {state}")
            time.sleep(1)
        raise RuntimeError(f"Readiness timeout for {service}")

    def snapshot(self, phase):
        value = json.loads(
            self.execute(
                "px4-sim",
                "python3",
                "-m",
                "aerotwin_experiments.m3_container",
                "snapshot",
                timeout=50,
            )
        )
        value.update(
            phase=phase, host_wall_ns=time.time_ns(), host_monotonic_ns=time.monotonic_ns()
        )
        self.checkpoints.append(value)
        save(self.directory / "checkpoints.json", self.checkpoints)
        return value

    def control(self, request):
        response = self.execute(
            "px4-sim",
            "gz",
            "service",
            "-s",
            "/world/default/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            request,
        )
        if not re.search(r"^data:\s*true\s*$", response, re.MULTILINE):
            raise RuntimeError(f"Gazebo rejected control: {response}")

    def phase(self, name, seconds=3):
        first = self.snapshot(name + "-start")
        time.sleep(seconds)
        last = self.snapshot(name + "-end")
        observer = json.loads((self.directory / "observer-ready.json").read_text())
        sim_clock, ros_clock = first["clock_identity"], observer["clock_identity"]
        if (
            sim_clock["boot_id"] != ros_clock["boot_id"]
            or sim_clock["offsets"] != ros_clock["offsets"]
        ):
            raise RuntimeError("Receipt and simulator kernel clock origins or offsets differ")
        observed = [
            v
            for v in samples(self.directory / "samples.jsonl")
            if first["monotonic_ns"] <= v["receipt_monotonic_ns"] <= last["monotonic_ns"]
        ]
        return {
            "start": first,
            "end": last,
            "messages": timestamp_changes(observed),
            "first_samples": observed[:3],
            "last_samples": observed[-3:],
        }

    def run(self):
        result = {
            "identity": self.identity,
            "transport": "NOT_RUN",
            "clocks": {},
            "clock_mapping": "BLOCKED",
            "coupled_reset": "BLOCKED",
        }
        try:
            save(
                self.directory / "deployment.json",
                {
                    "image": self.env["AEROTWIN_RUNTIME_IMAGE"],
                    "project": self.identity,
                    "compose": self.call("config", "--format", "json"),
                },
            )
            inventory = json.loads(
                self.call(
                    "run",
                    "--rm",
                    "--no-deps",
                    "-T",
                    "--entrypoint",
                    "python3",
                    "xrce-agent",
                    "-m",
                    "aerotwin_experiments.m3_container",
                    "inventory",
                    timeout=45,
                )
            )
            if inventory["architecture"] != "aarch64" or inventory["ros_distribution"] != "jazzy":
                raise RuntimeError(f"Unexpected platform: {inventory}")
            save(self.directory / "inventory.json", inventory)
            self.call("up", "-d", *SERVICES, timeout=90)
            self.ready("agent-ready.json", "xrce-agent")
            result["sim"] = self.ready("sim-ready.json", "px4-sim")
            result["observer"] = self.ready("observer-ready.json", "ros-observer")
            result["topology"] = {service: self.inspect(service) for service in SERVICES}
            normal = self.phase("normal", 4)
            result["clocks"]["normal"] = normal
            valid = ordered_window(samples(self.directory / "samples.jsonl"))
            if not valid:
                raise RuntimeError("No three consecutive valid ordered decoded messages")
            result["transport"] = "PASS"
            result["accepted_samples"] = valid
            self.control("pause: true")
            result["clocks"]["pause"] = self.phase("pause", 4)
            self.control("pause: false")
            result["clocks"]["resume"] = self.phase("resume", 4)
            self.control("pause: true")
            result["before_reset"] = self.snapshot("before-reset")
            self.control("reset: {time_only: true}")
            # Source telemetry can stall after rollback. Each probe remains bounded.
            result["clocks"]["time_only_reset"] = self.phase("reset", 4)
        except KeyboardInterrupt:
            result["error"] = "Interrupted; owned service cleanup requested"
            result["interrupted"] = True
        except (OSError, ValueError, RuntimeError) as error:
            result["error"] = str(error)
        finally:
            result["cleanup"] = "FAIL"
            try:
                self.call("stop", "-t", "15", "ros-observer", "px4-sim", "xrce-agent", timeout=65)
                states = []
                for service in SERVICES:
                    ids = self.call("ps", "-aq", service)
                    if not ids:
                        continue
                    states.append(self.inspect(service)["State"])
                    (self.directory / f"{service}.log").write_text(
                        command("docker", "logs", ids, timeout=15)
                    )
                result["exit_states"] = states
                logs = {}
                for role in ("sim", "agent"):
                    path = self.directory / f"{role}-processes.json"
                    if path.exists():
                        logs[role] = json.loads(path.read_text())
                validate_cleanup(states, logs)
                if len(states) != 3:
                    raise RuntimeError("Expected three service exit states")
                result["cleanup"] = "PASS"
            except (OSError, ValueError, RuntimeError) as error:
                result["cleanup_error"] = str(error)
            for name, phase in result["clocks"].items():
                phase["status"] = clock_status(
                    phase, result.get("before_reset") if name == "time_only_reset" else None
                )
            # Transport acceptance also requires a live source and a stable publisher.
            if (self.directory / "publisher-change.json").exists():
                result["transport"] = "FAIL"
                result["publisher_change"] = json.loads(
                    (self.directory / "publisher-change.json").read_text()
                )
            save(self.directory / "report.json", result)
            # Remove only this UUID-scoped project after evidence is written.
            try:
                self.call("down", timeout=45)
                result["removed"] = True
            except RuntimeError as error:
                result["removed"] = False
                result["removal_error"] = str(error)
            save(self.directory / "report.json", result)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, help="Immutable recovered local image sha256 ID")
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument(
        "--compose",
        type=Path,
        default=Path(__file__).resolve().parents[4] / "simulation/docker/compose.yaml",
    )
    args = parser.parse_args(argv)

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", args.image):
        parser.error("--image requires an immutable local sha256 ID")
    destination = args.evidence.resolve()
    if destination.exists():
        parser.error("--evidence must be a new directory")
    destination.mkdir(parents=True)
    try:
        command("docker", "info", "--format", "{{.Architecture}}")
        image = json.loads(command("docker", "image", "inspect", args.image))[0]
        if image["Architecture"] != "arm64" or image["Id"] != args.image:
            raise RuntimeError("Snapshot identity or architecture mismatch")
        save(destination / "image.json", image)
        reports = []
        for number in (1, 2):
            directory = destination / f"session-{number}"
            directory.mkdir()
            reports.append(Session(args.compose.resolve(), args.image, directory).run())
            if reports[-1].get("interrupted"):
                break
        save(destination / "summary.json", reports)
        save(destination / "acceptance.json", acceptance(reports))
        return 0 if acceptance(reports)["multi_container_transport"] == "PASS" else 1
    except (OSError, ValueError, RuntimeError) as error:
        save(destination / "blocked.json", {"status": "BLOCKED", "error": str(error)})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
