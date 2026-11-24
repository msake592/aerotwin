"""Portable failure-path tests. Live robotics observations are recorded separately."""

import json
import signal
from types import SimpleNamespace

import pytest
from aerotwin_experiments import m3_container as container
from aerotwin_experiments import m3_docker as docker
from aerotwin_experiments.m3_odometry_probe import odometry_sample
from aerotwin_experiments.m3_preflight import Check


def sample(timestamp, **changes):
    fields = dict(
        timestamp=timestamp,
        timestamp_sample=timestamp,
        pose_frame=1,
        velocity_frame=1,
        position=[0.0] * 3,
        velocity=[0.0] * 3,
        q=[1.0, 0.0, 0.0, 0.0],
        reset_counter=0,
    )
    return odometry_sample(SimpleNamespace(**(fields | changes)))


@pytest.mark.parametrize("detail", ["Docker daemon unavailable", "Timed out after 15s"])
def test_docker_failure_propagates(monkeypatch, detail):
    monkeypatch.setattr(docker, "probe", lambda *a, **kw: Check("docker", "FAIL", detail))
    with pytest.raises(RuntimeError, match=detail):
        docker.command("docker", "info")


def test_missing_artifact_fails_inventory(monkeypatch):
    monkeypatch.setattr(
        container,
        "checked",
        lambda *a, **kw: container.REVISIONS[
            "px4"
            if str(a[2]) == str(container.PX4)
            else "agent"
            if str(a[2]) == "/work/dds/agent"
            else "px4_msgs"
        ],
    )
    monkeypatch.setattr(
        container, "file_hash", lambda _: (_ for _ in ()).throw(FileNotFoundError())
    )
    with pytest.raises(FileNotFoundError):
        container.inventory()


def test_wrong_revision_stops_before_hashing(monkeypatch):
    monkeypatch.setattr(container, "checked", lambda *a, **kw: "wrong")
    with pytest.raises(RuntimeError, match="Revision mismatch"):
        container.inventory()


def test_artifact_hash_mismatch(monkeypatch):
    sources = {
        str(container.PX4): "px4",
        "/work/dds/agent": "agent",
        str(container.MSGS): "px4_msgs",
    }
    monkeypatch.setattr(
        container, "checked", lambda *a, **kw: container.REVISIONS[sources[str(a[2])]]
    )
    monkeypatch.setattr(container, "file_hash", lambda _: "bad")
    with pytest.raises(RuntimeError, match="Artifact identity"):
        container.inventory()


@pytest.mark.parametrize("name", ["sim-ready.json", "observer-ready.json"])
def test_readiness_exit_detects_agent_or_discovery_failure(tmp_path, monkeypatch, name):
    session = docker.Session(tmp_path / "compose.yaml", "sha256:abc", tmp_path)
    monkeypatch.setattr(session, "inspect", lambda _: {"State": {"Running": False, "ExitCode": 1}})
    with pytest.raises(RuntimeError, match="exited before readiness"):
        session.ready(name, "service", timeout=1)


def test_readiness_timeout_is_bounded(tmp_path, monkeypatch):
    session = docker.Session(tmp_path / "compose.yaml", "sha256:abc", tmp_path)
    with pytest.raises(RuntimeError, match="Readiness timeout"):
        session.ready("missing.json", "service", timeout=0)


def test_invalid_decoded_messages_do_not_pass():
    values = [sample(i, position=[float("nan")] * 3) for i in (1, 2, 3)]
    assert not docker.ordered_window(values)


def test_ordering_and_discontinuity():
    values = [sample(i) for i in (3, 2, 2, 4)]
    assert not docker.ordered_window(values)
    assert docker.timestamp_changes(values) == {
        "count": 4,
        "backward_publication": 1,
        "duplicate_publication": 1,
        "backward_acquisition": 1,
        "invalid": 0,
    }


def test_valid_window_requires_one_reset_counter():
    values = [sample(1), sample(2, reset_counter=1), sample(3), sample(4), sample(5)]
    assert docker.ordered_window(values) == values[-3:]


def test_partial_last_json_line_preserves_complete_observations(tmp_path):
    path = tmp_path / "samples.jsonl"
    path.write_text('{"a": 1}\n{"a":')
    assert docker.samples(path) == [{"a": 1}]
    path.write_text('{broken}\n{"a": 1}\n')
    with pytest.raises(json.JSONDecodeError):
        docker.samples(path)


@pytest.mark.parametrize(
    "state",
    [
        {"Running": True, "OOMKilled": False, "ExitCode": 0},
        {"Running": False, "OOMKilled": True, "ExitCode": 137},
        {"Running": False, "OOMKilled": False, "ExitCode": 1},
    ],
)
def test_unclean_container_fails(state):
    with pytest.raises(RuntimeError, match="Unclean service"):
        docker.validate_cleanup([state], {})


def test_cleanup_requires_all_owned_process_logs():
    with pytest.raises(RuntimeError, match="Missing sim"):
        docker.validate_cleanup([], {})


def test_forced_cleanup_fails():
    log = {"forced_cleanup": True, "exit_code": -signal.SIGKILL}
    with pytest.raises(RuntimeError, match="Unclean child"):
        docker.validate_cleanup([], {"sim": {"px4": log}})


def test_cleanup_accepts_sigterm_agent():
    log = {"forced_cleanup": False, "exit_code": -signal.SIGTERM}
    docker.validate_cleanup(
        [{"Running": False, "OOMKilled": False, "ExitCode": 0}],
        {"sim": {"px4": {"forced_cleanup": False, "exit_code": 0}}, "agent": {"agent": log}},
    )


def test_rejected_reset_command_fails(tmp_path, monkeypatch):
    session = docker.Session(tmp_path / "compose.yaml", "sha256:abc", tmp_path)
    monkeypatch.setattr(session, "execute", lambda *a, **kw: "data: false")
    with pytest.raises(RuntimeError, match="rejected control"):
        session.control("reset: {time_only: true}")


def test_phase_compares_verified_linux_monotonic_namespace(tmp_path, monkeypatch):
    session = docker.Session(tmp_path / "compose.yaml", "sha256:abc", tmp_path)
    observations = [
        dict(sample(1), receipt_monotonic_ns=15),
        dict(sample(2), receipt_monotonic_ns=25),
    ]
    (tmp_path / "samples.jsonl").write_text(
        "".join(json.dumps(value) + "\n" for value in observations)
    )
    (tmp_path / "observer-ready.json").write_text(
        '{"clock_identity": {"boot_id": "same", "offsets": ["monotonic", "0", "0"]}}'
    )
    points = iter(
        [
            {
                "clock_identity": {"boot_id": "same", "offsets": ["monotonic", "0", "0"]},
                "monotonic_ns": 10,
            },
            {
                "clock_identity": {"boot_id": "same", "offsets": ["monotonic", "0", "0"]},
                "monotonic_ns": 20,
            },
        ]
    )
    monkeypatch.setattr(session, "snapshot", lambda _: next(points))
    result = session.phase("pause", seconds=0)
    assert result["messages"]["count"] == 1
    assert result["first_samples"][0]["timestamp_us"] == 1


def test_distinct_monotonic_namespaces_block_clock_comparison(tmp_path, monkeypatch):
    session = docker.Session(tmp_path / "compose.yaml", "sha256:abc", tmp_path)
    (tmp_path / "observer-ready.json").write_text(
        '{"clock_identity": {"boot_id": "same", "offsets": ["monotonic", "0", "0"]}}'
    )
    monkeypatch.setattr(
        session,
        "snapshot",
        lambda _: {"clock_identity": {"boot_id": "different", "offsets": ["monotonic", "0", "0"]}},
    )
    with pytest.raises(RuntimeError, match="origins or offsets differ"):
        session.phase("pause", seconds=0)


def test_reset_contract_requires_observed_rollback_and_progression():
    phase = {
        "start": {"phase": "reset-start", "gazebo_stats": [[1, False], [2, False]]},
        "end": {"gazebo_stats": [[3, False], [4, False]]},
        "messages": {"count": 0},
    }
    before = {"gazebo_stats": [[10, True], [10, True]]}
    assert docker.clock_status(phase, before) == "PASS"
    assert docker.clock_status(phase, {"gazebo_stats": [[0, True], [0, True]]}) == "FAIL"
    phase["end"]["gazebo_stats"] = [[3, True], [3, False]]
    assert docker.clock_status(phase, before) == "FAIL"


def test_pause_contract_requires_all_stats_to_hold():
    phase = {
        "start": {"phase": "pause-start", "gazebo_stats": [[1, True], [1, True]]},
        "end": {"gazebo_stats": [[1, True], [1, True]]},
        "messages": {"count": 0},
    }
    assert docker.clock_status(phase) == "PASS"
    phase["end"]["gazebo_stats"] = [[2, True], [2, True]]
    assert docker.clock_status(phase) == "FAIL"


def test_main_records_missing_daemon_without_starting_sessions(tmp_path, monkeypatch):
    monkeypatch.setattr(
        docker,
        "command",
        lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("Docker daemon unavailable")),
    )
    destination = tmp_path / "evidence"
    assert docker.main(["--image", "sha256:" + "1" * 64, "--evidence", str(destination)]) == 2
    assert json.loads((destination / "blocked.json").read_text())["status"] == "BLOCKED"
    assert not (destination / "session-1").exists()


def test_command_preserves_empty_success(monkeypatch):
    monkeypatch.setattr(
        docker, "probe", lambda *a, **kw: Check("docker", "PASS", "Exited successfully")
    )
    assert docker.command("docker", "compose", "ps", "-aq", "missing") == ""


def test_partial_sessions_cannot_pass_transport_or_restart():
    report = {"transport": "PASS", "cleanup": "PASS", "removed": True, "clocks": {}}
    result = docker.acceptance([report])
    assert result["multi_container_transport"] == "FAIL"
    assert result["controlled_full_restart"] == "NOT_RUN"
    assert result["m34_readiness"] == "BLOCKED"


def test_timestamp_discontinuity_fails_normal_clock_gate():
    phase = {
        "start": {"phase": "normal-start", "gazebo_stats": [[1, False], [2, False]]},
        "end": {"gazebo_stats": [[3, False], [4, False]]},
        "messages": {"count": 3, "backward_acquisition": 1},
    }
    assert docker.clock_status(phase) == "FAIL"


def test_udp_peers_are_resolved_at_startup(tmp_path, monkeypatch):
    import xml.etree.ElementTree as xml

    monkeypatch.setattr(container, "EVIDENCE", tmp_path)
    monkeypatch.setattr(container.socket, "gethostbyname", lambda peer: "172.20.0.8")
    monkeypatch.setenv("ROS_DOMAIN_ID", "55")
    monkeypatch.setenv("FASTDDS_DEFAULT_PROFILES_FILE", "")
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    container.dds_profile("xrce-agent")
    profile = xml.fromstring((tmp_path / "observer-dds-profile.xml").read_text())
    ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    assert profile.find(".//d:address", ns).text == "172.20.0.8"
    assert profile.find(".//d:port", ns).text == "21160"
    assert profile.find(".//d:useBuiltinTransports", ns).text == "false"


def test_inventory_failure_prevents_runtime_start(tmp_path, monkeypatch):
    session = docker.Session(tmp_path / "compose.yaml", "sha256:abc", tmp_path)
    commands = []

    def call(*args, **kwargs):
        commands.append(args)
        if args[0] == "config":
            return "{}"
        if args[0] == "run":
            raise RuntimeError("Artifact identity mismatch")
        if args[0] == "ps":
            return ""
        return ""

    monkeypatch.setattr(session, "call", call)
    result = session.run()
    assert "Artifact identity mismatch" in result["error"]
    assert not any(args[0] == "up" for args in commands)
    assert result["transport"] == "NOT_RUN"
    assert result["removed"]
