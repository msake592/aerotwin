import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from aerotwin_experiments import m3_odometry_probe as odometry
from aerotwin_experiments import m3_runtime as runtime
from aerotwin_experiments.m3_preflight import Check


def test_plan_reports_pinned_candidates(capsys):
    assert runtime.main(["plan"]) == 0
    assert json.loads(capsys.readouterr().out)["revisions"] == runtime.REVISIONS


@pytest.mark.parametrize(("system", "arch"), [("Darwin", "arm64"), ("Linux", "aarch64")])
def test_unsupported_reference_platform(monkeypatch, capsys, system, arch):
    monkeypatch.setattr(runtime.preflight.platform, "system", lambda: system)
    monkeypatch.setattr(runtime.preflight.platform, "machine", lambda: arch)
    monkeypatch.setattr(runtime.preflight, "os_release", lambda: ("ubuntu", "24.04"))
    assert runtime.main(["live"]) == 2
    report = json.loads(capsys.readouterr().out)
    assert all(
        row["status"] == "BLOCKED" for row in report["checks"] if row["name"] in runtime.LIVE_GATES
    )


def test_package_inventory_keeps_only_installed_revisions(monkeypatch):
    monkeypatch.setattr(
        runtime,
        "probe",
        lambda *a, **kw: Check("dpkg", "FAIL", "Exit 1: ii \tgz-harmonic\t1.0\nrc \told\t2"),
    )
    assert runtime.package_inventory() == {"gz-harmonic": "1.0"}


def test_stats_read_binary_time_boundaries(monkeypatch):
    monkeypatch.setattr(
        runtime,
        "probe",
        lambda *a, **kw: Check(
            "stats",
            "PASS",
            "sim_time {\n  sec: 1\n  nsec: 2\n}\n\nsim_time {\n}\npaused: true",
        ),
    )
    assert runtime.stats("preflight", {}) == [(1_000_000_002, False), (0, True)]


@pytest.mark.parametrize("output", ["", "sim_time { nsec: 1000000000 }", "sim_time { sec: -1 }"])
def test_stats_rejects_missing_or_invalid_samples(monkeypatch, output):
    monkeypatch.setattr(runtime, "probe", lambda *a, **kw: Check("stats", "PASS", output))
    with pytest.raises(RuntimeError):
        runtime.stats("preflight", {})


def test_clock_exercises_pause_reset_and_resume(monkeypatch, tmp_path):
    from contextlib import nullcontext

    snapshots = iter([[(1, False), (2, False)], [(3, False), (4, False)]])
    states = iter([[(2, True), (2, True)], [(0, True), (0, True)]])
    controls = []
    monkeypatch.setattr(runtime, "owned_process", lambda *a: nullcontext((object(), None)))
    monkeypatch.setattr(runtime, "wait_for_progress", lambda *a: next(snapshots))
    monkeypatch.setattr(runtime, "stats", lambda *a: next(states))
    monkeypatch.setattr(runtime, "control", lambda req, env: controls.append(req))
    evidence = {}
    runtime.gazebo_clock(tmp_path, {}, evidence)
    assert controls == ["pause: true", "reset: {time_only: true}", "pause: false"]
    assert evidence["reset"] == [(0, True), (0, True)]


def test_manifest_blocker_prevents_probes_and_launch(monkeypatch):
    monkeypatch.setattr(runtime, "manifest_checks", lambda *_: [Check("source", "BLOCKED", "bad")])

    def unexpected(*_):
        pytest.fail("Started runtime checks despite an invalid manifest")

    monkeypatch.setattr(runtime, "inventory_checks", unexpected)
    monkeypatch.setattr(runtime, "run_gate", unexpected)
    checks = runtime.live_report({}, {}, 55, 18888)
    assert [check["name"] for check in checks if check["status"] == "BLOCKED"] == [
        "source",
        *runtime.LIVE_GATES,
    ]


def test_missing_ros_interface_blocks_both_runs(monkeypatch):
    monkeypatch.setattr(runtime, "manifest_checks", lambda *_: [])
    monkeypatch.setattr(
        runtime, "inventory_checks", lambda *_: [Check("VehicleOdometry", "FAIL", "missing")]
    )
    checks = runtime.live_report({}, {}, 55, 18888)
    assert checks[0]["status"] == "FAIL"
    assert all(check["status"] == "BLOCKED" for check in checks[-4:])


def test_repetitions_use_fresh_names_and_keep_second_failure(monkeypatch):
    monkeypatch.setattr(runtime, "manifest_checks", lambda *_: [])
    monkeypatch.setattr(runtime, "inventory_checks", lambda *_: [])
    monkeypatch.setattr(runtime, "gazebo_clock", lambda *_: None)
    seen = []

    def sitl(work, env, paths, topic, evidence):
        assert work.is_dir()
        seen.append((work, env["GZ_PARTITION"], topic))
        if len(seen) == 2:
            raise RuntimeError("second execution failed")

    monkeypatch.setattr(runtime, "sitl_run", sitl)
    checks = runtime.live_report({}, {}, 55, 18888)
    assert [check["status"] for check in checks[-4:]] == ["PASS", "PASS", "FAIL", "NOT_RUN"]
    assert seen[0][1:] != seen[1][1:]
    assert all(not work.exists() for work, _, _ in seen)


@pytest.mark.skipif(os.name != "posix", reason="POSIX process groups")
def test_owned_process_cleanup_after_interruption(tmp_path):
    logs = {}
    with pytest.raises(KeyboardInterrupt):
        with runtime.owned_process(
            (sys.executable, "-c", "import time; time.sleep(60)"), tmp_path, os.environ.copy(), logs
        ) as (child, _):
            raise KeyboardInterrupt
    assert child.poll() is not None
    assert logs[sys.executable]["exit_code"] == child.returncode


def test_owned_process_reports_launch_failure(tmp_path):
    row = runtime.run_gate("launch", lambda evidence: launch_nonexistent(tmp_path, evidence))
    assert row["status"] == "FAIL"
    assert "nonexistent-test-binary" in row["detail"]


def launch_nonexistent(tmp_path: Path, evidence):
    with runtime.owned_process(
        (str(tmp_path / "nonexistent-test-binary"),), tmp_path, os.environ.copy(), evidence
    ):
        pytest.fail("Unexpected process")


@pytest.mark.parametrize(
    ("compatibility", "expected"), [("OK", True), ("WARNING", True), ("ERROR", False)]
)
def test_qos_gate(compatibility, expected):
    assert odometry.qos_allows_observation(compatibility) is expected


def sample(timestamp, **updates):
    attributes = dict(
        timestamp=timestamp,
        timestamp_sample=timestamp - 1,
        pose_frame=1,
        velocity_frame=3,
        q=[1, 0, 0, 0],
        position=[0, 1, 2],
        velocity=[0, 0, 0],
        reset_counter=0,
    )
    return odometry.odometry_sample(SimpleNamespace(**(attributes | updates)))


def test_odometry_requires_three_ordered_valid_samples():
    samples = [sample(i) for i in range(2, 5)]
    assert odometry.sample_series_valid(samples)
    assert not odometry.sample_series_valid(samples[:2])
    assert not odometry.sample_series_valid(samples[::-1])
    assert not odometry.sample_series_valid([*samples[:2], sample(4, reset_counter=1)])
    assert not sample(4, position=[0, float("nan"), 2])["valid"]
    assert not sample(4, pose_frame=0)["valid"]
    assert not sample(4, timestamp_sample=5)["valid"]


def test_probe_timeout_remains_a_failure():
    if os.name != "posix":
        pytest.skip("POSIX process groups")
    result = runtime.probe("timeout", sys.executable, "-c", "import time;time.sleep(60)", timeout=1)
    assert result.status == "FAIL"
    assert "Timed out" in result.detail


def test_live_requires_operator_to_confirm_isolation(monkeypatch, capsys):
    monkeypatch.setattr(
        runtime.preflight, "profile_checks", lambda _: [Check("platform", "PASS", "Noble")]
    )
    assert runtime.main(["live"]) == 2
    report = json.loads(capsys.readouterr().out)
    assert report["checks"][0]["name"] == "Host isolation"
    assert all(check["status"] == "BLOCKED" for check in report["checks"])
