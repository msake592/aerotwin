import subprocess
from pathlib import Path

import pytest
from aerotwin_experiments.timing_sensitivity import configurations, execute

BINARY = (
    Path(__file__).resolve().parents[2]
    / "build/dev-debug/examples/timing_sensitivity/aerotwin_timing_sensitivity"
)


def request(mode, error=0, motion="cv", seed=42):
    return dict(mode=mode, error=error, motion=motion, seed=seed)


@pytest.fixture
def binary():
    if not BINARY.is_file():
        pytest.skip("Build the C++ timing_sensitivity experiment target")
    return BINARY


def test_configurations_are_deterministic_and_cover_sweep():
    assert list(configurations()) == list(configurations())
    assert {row["motion"] for row in configurations()} == {"stationary", "cv", "accelerating"}
    assert any(row["mode"] == "stream" and row["error"] == -100_000_000 for row in configurations())


@pytest.mark.parametrize("motion", ["stationary", "cv", "accelerating"])
def test_zero_and_common_offset_invariance(binary, motion):
    rows = execute(
        binary,
        [request("baseline", motion=motion)]
        + [
            request(mode, motion=motion)
            for mode in ("stream", "jitter", "drift", "acquisition", "latency")
        ]
        + [request("common", value, motion=motion) for value in (-100_000_000, 100_000_000)],
    )
    for row in rows:
        assert row["position_rmse"] == rows[0]["position_rmse"]
        assert row["velocity_rmse"] == rows[0]["velocity_rmse"]


def test_jitter_and_results_reproduce(binary):
    configs = [request("baseline"), request("jitter", 500_000)]
    assert execute(binary, configs) == execute(binary, configs)
    row = execute(binary, configs)[1]
    assert -500_000 <= row["min_perturbation_ns"] <= row["max_perturbation_ns"] <= 500_000


def test_drift_accumulation(binary):
    row = execute(binary, [request("drift", 250)])[0]
    assert row["max_perturbation_ns"] == 1_000_000
    assert execute(binary, [request("drift", -250)])[0]["min_perturbation_ns"] == -1_000_000


def test_latency_preserves_acquisition_and_oos_rejects(binary):
    rows = execute(binary, [request("baseline"), request("latency", 100_000_000), request("oos")])
    assert rows[1]["position_rmse"] == rows[0]["position_rmse"]
    assert rows[2]["ordering_violations"] == 1
    assert rows[2]["rejected"] == 1


def test_acquisition_shift_differs_from_delivery(binary):
    rows = execute(
        binary,
        [request("baseline"), request("acquisition", 100_000_000), request("latency", 100_000_000)],
    )
    assert rows[1]["position_rmse"] != rows[2]["position_rmse"]


def test_ground_truth_only_scores_or_generates_measurements(binary):
    rows = execute(binary, [request("baseline"), request("score_only")])
    assert rows[0]["mean_nis"] == rows[1]["mean_nis"]
    assert rows[0]["mean_position_trace"] == rows[1]["mean_position_trace"]
    assert rows[0]["mean_velocity_trace"] == rows[1]["mean_velocity_trace"]
    assert rows[0]["velocity_rmse"] == rows[1]["velocity_rmse"]
    assert rows[1]["position_rmse"] > rows[0]["position_rmse"] + 90


@pytest.mark.parametrize(
    "line",
    [
        "cv unknown 0 42",
        "unknown baseline 0 42",
        "cv drift 25001 42",
        "cv baseline 0 -1",
        "cv baseline 0 4294967296",
        "cv jitter 100000001 42",
        "cv latency -1 42",
        "cv",
        "cv baseline 0 42 extra",
    ],
)
def test_invalid_inputs(binary, line):
    assert (
        subprocess.run(
            [str(binary)], input=line + "\n", text=True, capture_output=True, timeout=10
        ).returncode
        == 1
    )


def test_reordering_metadata_records_complete_delivery_delay(binary):
    row = execute(binary, [request("oos", 100_000_000)])[0]
    assert row["delivery_delay_ns"] == 2_100_000_000
    assert row["rejected"] == row["ordering_violations"] == 1
