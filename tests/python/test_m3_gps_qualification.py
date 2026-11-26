import json
import subprocess
import sys
from copy import deepcopy
from dataclasses import replace

import pytest
from aerotwin_experiments.m3_clock import MAX_NS, Mapping, Observation, State
from aerotwin_experiments.m3_gps_qualification import (
    PARAMETERS,
    SOURCE_TOPIC,
    WIRE_TOPIC,
    compare,
    payload,
    protocol,
)


def context():
    return dict(
        epoch="run1",
        generation=1,
        source_publisher="uorb-instance0",
        wire_publisher="gid1",
        device_id=123,
        receiver_instances=[0],
        selected_receiver=0,
        parameters=dict(PARAMETERS),
    )


def row(index=0, side="source"):
    return dict(
        epoch="run1",
        generation=1,
        publisher="uorb-instance0" if side == "source" else "gid1",
        topic=SOURCE_TOPIC if side == "source" else WIRE_TOPIC,
        receiver_instance=0,
        device_id=123,
        clock="px4_hrt_us",
        timestamp_us=10_000_000 + index * 1_000_000 // 30,
        timestamp_sample_us=10_000_000 + index * 1_000_000 // 30,
        latitude_deg=47 + index * 1e-8,
        longitude_deg=8.0,
        altitude_msl_m=10.0,
        altitude_ellipsoid_m=10.0,
        vel_n_m_s=0.1,
        vel_e_m_s=0.0,
        vel_d_m_s=-0.0,
        fix_type=3,
        satellites_used=16,
        vel_ned_valid=True,
    )


def test_frozen_protocol_uses_sensor_rate_and_requires_reference():
    p = protocol()
    assert p["configured_sensor_rate_hz"] == 30
    assert p["minimum_source_wire_and_matched_count"] == 1800
    assert p["steady_duration_ns"] == 60_000_000_000
    assert p["maximum_live_attempts"] == p["required_sessions"] == 2
    assert p["registration_budget_ns"] == 500_000
    assert not p["independent_reference_available"]


def test_payload_identity_excludes_timestamps_and_retains_signed_zero():
    original = row()
    changed = original | dict(timestamp_us=999, timestamp_sample_us=998)
    assert payload(original) == payload(changed)
    assert payload(original) != payload(original | dict(vel_d_m_s=0.0))


def test_exact_preservation_does_not_qualify_acquisition():
    result = compare([row()], [row(side="wire")], context())
    assert result["timestamp_preservation"] == "PASS"
    assert result["pairs"][0]["acquisition_difference_ns"] == 0
    assert result["pairs"][0]["publication_difference_ns"] == 0
    assert result["registration_uncertainty_ns"] is None
    assert result["gps_qualification"] == "BLOCKED"
    assert not result["mapping_valid"]


@pytest.mark.parametrize("field", ["timestamp_us", "timestamp_sample_us"])
def test_one_microsecond_difference_fails_exact_equality(field):
    wire = row(side="wire")
    wire[field] += 1 if field == "timestamp_us" else -1
    result = compare([row()], [wire], context())
    assert result["timestamp_preservation"] == "FAIL"
    assert result["differing_pairs"] == 1


def test_ambiguous_payloads_reject_all_correspondences():
    source = [row(), row() | dict(timestamp_us=10_033_333, timestamp_sample_us=10_033_333)]
    wire = [row(side="wire")]
    result = compare(source, wire, context())
    assert result["matched"] == 0
    assert result["ambiguous_source_wire"] == [2, 1]
    assert result["unmatched_source_wire"] == [2, 1]


@pytest.mark.parametrize(
    "changes",
    [
        {"epoch": "old-run"},
        {"generation": 0},
        {"generation": True},
        {"publisher": "changed"},
        {"receiver_instance": 1},
        {"device_id": 0},
        {"timestamp_sample_us": 0},
        {"timestamp_us": MAX_NS // 1000 + 1},
        {"timestamp_sample_us": True},
        {"clock": "gps_utc"},
        {"latitude_deg": float("nan")},
        {"vel_n_m_s": 1e300},
        {"fix_type": 2},
        {"fix_type": 255},
        {"vel_ned_valid": False},
    ],
)
def test_invalid_stale_or_changed_records_rejected(changes):
    result = compare([row()], [row(side="wire") | changes], context())
    assert result["invalid_source_wire"][1] == 1
    assert result["matched"] == 0
    assert not result["mapping_valid"]


def test_reversed_input_order_is_not_hidden_by_payload_join():
    result = compare([row(0), row(1)], [row(1, "wire"), row(0, "wire")], context())
    assert result["ordering_source_wire"] == [0, 1]
    assert result["timestamp_preservation"] == "FAIL"


def test_unmatched_and_duplicate_wire_records_are_reported():
    result = compare([row()], [row(side="wire"), row(side="wire"), row(1, "wire")], context())
    assert result["unmatched_source_wire"] == [1, 3]
    assert result["ambiguous_source_wire"] == [1, 2]


def test_native_rate_coverage_and_reproducible_results():
    source, wire = [row(i) for i in range(1801)], [row(i, "wire") for i in range(1801)]
    a = compare(source, wire, context())
    assert a == compare(source, wire, context())
    assert a["observation_coverage"] == "PASS"
    assert a["source_wire_spans_ns"] == [60_000_000_000] * 2
    assert a["gps_qualification"] == "BLOCKED"
    assert compare(source[:61], wire[:61], context())["observation_coverage"] == "BLOCKED"


@pytest.mark.parametrize(
    "change", [{"UXRCE_DDS_SYNCT": 1}, {"SENS_GPS_MASK": 7}, {"SENS_GPS_PRIME": 1}]
)
def test_adjusted_or_blended_context_rejected(change):
    c = deepcopy(context())
    c["parameters"].update(change)
    with pytest.raises(ValueError):
        compare([], [], c)


def test_existing_mapping_rejects_buffered_old_epoch_and_generation():
    mapping = Mapping("run1")
    mapping.begin("gid1", "px4_hrt_us")
    mapping.validate(epoch="run1", generation=1, offset_ns=0, uncertainty_ns=0)
    old = Observation("run1", 1, "gid1", "fix1", 10_000_000, "px4_hrt_us", 1)
    assert mapping.convert(old) == 10_000_000_000
    mapping.reconnect()
    assert mapping.state == State.INVALIDATED
    with pytest.raises(ValueError):
        mapping.convert(replace(old, identity="fix2", timestamp_us=10_033_333))
    mapping.restart("run2")
    mapping.begin("gid2", "px4_hrt_us")
    mapping.validate(epoch="run2", generation=mapping.generation, offset_ns=0, uncertainty_ns=0)
    with pytest.raises(ValueError):
        mapping.convert(old)
    fresh = replace(old, epoch="run2", generation=mapping.generation, publisher="gid2")
    mapping.convert(fresh)
    with pytest.raises(ValueError):
        mapping.convert(fresh)
    assert mapping.state == State.INVALIDATED


def test_multiple_receiver_context_stops_before_matching():
    c = context() | {"receiver_instances": [0, 1]}
    with pytest.raises(ValueError):
        compare([], [], c)


def test_cli_writes_blocked_preflight_without_signalling_readiness(tmp_path):
    output = tmp_path / "preflight.json"
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "aerotwin_experiments.m3_gps_qualification",
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 2
    report = json.loads(output.read_text())
    assert report["live_attempts"] == 0
    assert report["gps_qualification"] == "BLOCKED"
