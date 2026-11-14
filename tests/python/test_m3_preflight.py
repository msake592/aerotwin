import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
from aerotwin_experiments import m3_preflight as preflight


@pytest.mark.parametrize(
    ("output", "status"),
    [
        ("Gazebo Sim, version 8.15.0\nCopyright (C) 2018 Open Source Robotics Foundation.", "PASS"),
        ("8.9.0", "PASS"),
        ("Gazebo Sim, version 10.0.0", "BLOCKED"),
        ("unexpected output with 8.1.0", "FAIL"),
    ],
)
def test_gazebo_version(output, status):
    assert preflight.gaz_sim_version(preflight.Check("gz", "PASS", output)).status == status


def test_missing_tool_does_not_start_subprocess(monkeypatch):
    monkeypatch.setattr(preflight.shutil, "which", lambda _: None)
    assert preflight.probe("Gazebo", "gz", "sim", "--version").status == "BLOCKED"


@pytest.mark.skipif(os.name != "posix", reason="POSIX process groups")
def test_command_exit_and_timeout():
    result = preflight.probe("exit", sys.executable, "-c", "print('problem'); raise SystemExit(7)")
    assert result.status == "FAIL"
    assert result.detail == "Exit 7: problem"
    timeout = preflight.probe("timeout", sys.executable, "-c", "while True: pass", timeout=1)
    assert timeout.status == "FAIL"
    assert timeout.detail == "Timed out after 1s"


def test_cleanup_kills_members_after_leader_exits(monkeypatch):
    calls = []

    class Process:
        pid = 42

        def wait(self, timeout=None):
            calls.append(("wait", timeout))
            if timeout:
                raise subprocess.TimeoutExpired("simulator", timeout)

    monkeypatch.setattr(preflight.os, "killpg", lambda pid, signum: calls.append((pid, signum)))
    preflight.stop_group(Process())
    assert calls == [
        (42, preflight.signal.SIGTERM),
        ("wait", 1),
        (42, preflight.signal.SIGKILL),
        ("wait", None),
    ]


def test_unsupported_profile_produces_blocked_json(monkeypatch, capsys):
    monkeypatch.setattr(preflight.platform, "system", lambda: "FreeBSD")
    monkeypatch.setattr(preflight.platform, "machine", lambda: "amd64")
    assert preflight.main(["--profile", "linux-integration", "--smoke", "--json"]) == 2
    report = json.loads(capsys.readouterr().out)
    assert report["profile"] == "linux-integration"
    assert report["exit_code"] == 2
    assert [check["status"] for check in report["checks"]] == ["BLOCKED", "SKIP"]


def test_noble_arm64_warns_about_harmonic(monkeypatch):
    monkeypatch.setattr(preflight.platform, "system", lambda: "Linux")
    monkeypatch.setattr(preflight.platform, "machine", lambda: "aarch64")
    monkeypatch.setattr(preflight, "os_release", lambda: ("ubuntu", "24.04"))
    assert [check.status for check in preflight.profile_checks("linux-integration")] == [
        "PASS",
        "WARN",
    ]


def test_smoke_skips_missing_or_unsourced_tools(monkeypatch):
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    monkeypatch.setattr(preflight.shutil, "which", lambda _: None)
    checks = preflight.linux_checks(True, None, None)
    results = {check.name: check.status for check in checks}
    assert results["Gazebo Harmonic"] == results["ROS 2 CLI"] == "BLOCKED"
    assert results["Gazebo server"] == results["ROS interfaces and discovery"] == "SKIP"


@pytest.mark.parametrize(
    ("logs", "server_status"),
    [
        ("World [preflight] initialized\nShutting down gz-sim-server", "PASS"),
        ("Shutting down gz-sim-server", "FAIL"),
        ("World [preflight] initialized\n[Err] plugin\nShutting down gz-sim-server", "FAIL"),
    ],
)
def test_runtime_probes_are_explicit_and_isolated(monkeypatch, logs, server_status):
    calls = []

    def probe(name, *command, **kwargs):
        calls.append(command)
        if name == "Gazebo Harmonic":
            return preflight.Check(name, "PASS", "Gazebo Sim, version 8.15.0")
        if name.startswith("Gazebo server"):
            assert kwargs["timeout"] == 30
            assert kwargs["env"]["GZ_PARTITION"].startswith("aerotwin_")
            assert '<world name="preflight">' in Path(command[-1]).read_text()
            return preflight.Check(name, "PASS", logs)
        if name == "DDS sample":
            assert kwargs["timeout"] == 15
            return preflight.Check(name, "PASS", "timestamp: 1234")
        return preflight.Check(name, "PASS", "ok")

    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    monkeypatch.setattr(preflight, "probe", probe)
    monkeypatch.setattr(preflight.shutil, "which", lambda tool: f"/usr/bin/{tool}")
    checks = preflight.linux_checks(False, None, None)
    assert checks[-1].status == "SKIP"
    assert len(calls) == 2
    checks = preflight.linux_checks(True, None, "/fmu/out/sensor_combined")
    assert (
        next(check.status for check in checks if check.name.startswith("Gazebo server"))
        == server_status
    )
    assert checks[-1].status == "PASS"
    assert calls[-1][-4:] == ("--once", "--no-daemon", "--qos-profile", "sensor_data")


def test_px4_revision_mismatch_cannot_launch_binary(monkeypatch):
    def probe(name, *command, **kwargs):
        if command[-1] == "--show-toplevel":
            return preflight.Check(name, "PASS", str(Path.cwd()))
        assert command[-2:] == ("rev-parse", "HEAD")
        return preflight.Check(name, "PASS", "unexpected-revision")

    monkeypatch.setattr(preflight, "probe", probe)
    assert preflight.px4_checks(Path.cwd())[0].status == "BLOCKED"
