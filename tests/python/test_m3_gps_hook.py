"""Synthetic evidence only; no simulation or qualification attempts."""

import copy
import hashlib
import json
import struct
import uuid
import zlib

import pytest
from aerotwin_experiments.m3_gps_hook import REVISION, assess, decode, header, main, payload_row
from aerotwin_experiments.m3_gps_qualification import PARAMETERS, payload


def metadata():
    return {
        "run_id": str(uuid.UUID(int=1)),
        "epoch": str(uuid.UUID(int=2)),
        "generation": 1,
        "sensor_topic": "/test",
        "frame_id": "gps",
        "upstream": REVISION,
        **{k + "_sha256": "ab" * 32 for k in ("patch", "binary", "base", "config", "gazebo")},
    }


def fixture(count=3):
    m = metadata()
    context = m | {
        "device_id": 42,
        "source_publisher": "selected",
        "wire_publisher": "dds",
        "raw_publisher": "raw",
        "receiver_instances": [0],
        "selected_receiver": 0,
        "parameters": PARAMETERS.copy(),
    }
    records, stages = [], [[], [], []]
    for i in range(count):
        base = {
            "latitude_deg": 47 + i * 0.001,
            "longitude_deg": 8.0,
            "altitude_msl_m": 400.0,
            "altitude_ellipsoid_m": 400.0,
            "vel_n_m_s": -0.0,
            "vel_e_m_s": 0.25,
            "vel_d_m_s": -0.5,
            "device_id": 42,
            "fix_type": 3,
            "satellites_used": 12,
            "vel_ned_valid": True,
        }
        stamp = 1_000_000 + i * 33_000
        records.append(
            {
                "callback_seq": i,
                "record_seq": i,
                "event_seq": i,
                "acquisition_ns": stamp * 1000,
                "assigned_us": stamp,
                "timestamp_us": stamp,
                "timestamp_sample_us": stamp,
                "device_id": 42,
                "receiver_instance": 0,
                "payload_hex": payload(base).hex(),
                "status": 2,
                "publish_ok": True,
                "vel_ned_valid": True,
                "host_brackets_ns": [0, 0, 0],
            }
        )
        for rows, topic, publisher in zip(
            stages,
            ("sensor_gps", "vehicle_gps_position", "/fmu/out/vehicle_gps_position"),
            ("raw", "selected", "dds"),
            strict=True,
        ):
            rows.append(
                base
                | {
                    "run_id": m["run_id"],
                    "epoch": m["epoch"],
                    "generation": 1,
                    "publisher": publisher,
                    "topic": topic,
                    "receiver_instance": 0,
                    "clock": "px4_hrt_us",
                    "timestamp_us": stamp,
                    "timestamp_sample_us": stamp,
                }
            )
    capture = {
        "metadata": m,
        "records": records,
        "counters": {
            "callbacks": count,
            "enqueued": count,
            "drained": count,
            "overflow": 0,
            "invalid": 0,
            "write_errors": 0,
            "high_water": 3,
            "shutdown_failed": 0,
        },
    }
    capture["overhead_ns"] = {"queue_max": 0, "callback_max": 0}
    return capture, *stages, context


def encode(capture):
    h = header(capture["metadata"])
    stream = bytearray()
    for r in capture["records"]:
        data = bytearray(192)
        data[:4] = b"EVT1"
        data[8:40] = h[480:]
        struct.pack_into(
            "<QQQqQQQIi",
            data,
            40,
            *(
                r[k]
                for k in (
                    "callback_seq",
                    "record_seq",
                    "event_seq",
                    "acquisition_ns",
                    "assigned_us",
                    "timestamp_us",
                    "timestamp_sample_us",
                    "device_id",
                    "receiver_instance",
                )
            ),
        )
        data[104:154] = bytes.fromhex(r["payload_hex"])
        data[154:157] = bytes((r["status"], r["publish_ok"], r["vel_ned_valid"]))
        stream.extend(data)
    footer = bytearray(128)
    footer[:8] = b"ATGPSF1\0"
    footer[8:40] = h[480:]
    struct.pack_into("<8Q", footer, 40, *capture["counters"].values())
    struct.pack_into("<I", footer, 104, zlib.crc32(footer, zlib.crc32(stream)))
    return h + stream + footer


def test_valid_decode_association_and_determinism():
    capture, raw, source, wire, context = fixture()
    decoded = decode(encode(capture))
    assert decoded == capture
    a = assess(decoded, raw, source, wire, context)
    assert a == assess(decoded, raw, source, wire, context)
    assert a["offline_association"] == "PASS"
    assert a["gps_qualification"] == a["m34"] == "BLOCKED"
    assert a["conditional_total_ns"] is None and a["live_attempts_consumed"] == 0
    assert struct.pack("<f", payload_row(decoded["records"][0])["vel_n_m_s"]) == bytes.fromhex(
        "00000080"
    )


@pytest.mark.parametrize(
    "kind",
    [
        "event_duplicate",
        "payload_duplicate",
        "missing_hook",
        "missing_raw",
        "missing_wire",
        "receiver",
        "stale",
        "epoch",
        "run",
        "ordering",
        "event_gap",
        "assigned_order",
        "time_changed",
        "invalid",
        "loss",
        "shutdown",
        "zero",
        "negative",
        "overflow",
        "publisher",
        "wrong_topic",
        "source_order",
        "ambiguous_wire",
    ],
)
def test_fail_closed(kind):
    capture, raw, source, wire, context = fixture()
    records = capture["records"]
    if kind == "event_duplicate":
        records[1]["event_seq"] = 0
    elif kind == "payload_duplicate":
        records[1]["payload_hex"] = records[0]["payload_hex"]
    elif kind == "missing_hook":
        records.pop()
    elif kind == "missing_raw":
        raw.pop()
    elif kind == "missing_wire":
        wire.pop()
    elif kind == "receiver":
        raw[0]["receiver_instance"] = 1
    elif kind == "stale":
        wire[0]["generation"] = 2
    elif kind == "epoch":
        wire[0]["epoch"] = "old"
    elif kind == "run":
        wire[0]["run_id"] = "old"
    elif kind == "ordering":
        records.reverse()
    elif kind == "event_gap":
        records[1]["event_seq"] = 9
    elif kind == "assigned_order":
        records[1]["assigned_us"] = 1
    elif kind == "time_changed":
        records[1]["timestamp_us"] += 1
    elif kind == "invalid":
        records[1]["status"] = 3
    elif kind == "loss":
        capture["counters"]["overflow"] = 1
    elif kind == "shutdown":
        capture["counters"]["shutdown_failed"] = 1
    elif kind == "zero":
        records[0]["acquisition_ns"] = 0
    elif kind == "negative":
        records[0]["acquisition_ns"] = -1
    elif kind == "overflow":
        records[0]["assigned_us"] = 1 << 63
    elif kind == "publisher":
        raw[0]["publisher"] = "other"
    elif kind == "wrong_topic":
        raw[0]["topic"] = "other"
    elif kind == "source_order":
        source.reverse()
    elif kind == "ambiguous_wire":
        wire.append(copy.deepcopy(wire[0]))
    result = assess(capture, raw, source, wire, context)
    assert result["offline_association"] == "FAIL"
    assert result["failures"] and result["gps_qualification"] == "BLOCKED"


@pytest.mark.parametrize("offset", [0, 8, 420, 480, 512, 520, 616, -1, -20, -120])
def test_corrupt_or_truncated_blob(offset):
    data = bytearray(encode(fixture()[0]))
    data[offset] ^= 1
    with pytest.raises(ValueError):
        decode(data)
    with pytest.raises(ValueError):
        decode(data[:-1])


@pytest.mark.parametrize(
    "key,value",
    [
        ("generation", True),
        ("generation", 0),
        ("upstream", "00" * 20),
        ("sensor_topic", "x" * 128),
        ("frame_id", "\0"),
        ("binary_sha256", "00" * 32),
        ("run_id", str(uuid.UUID(int=0))),
    ],
)
def test_invalid_context(key, value):
    m = metadata() | {key: value}
    with pytest.raises((ValueError, TypeError)):
        header(m)


def test_context_digest_and_uncertainties(tmp_path):
    capture, *rest = fixture()
    h = header(capture["metadata"])
    assert hashlib.sha256(h[:480]).digest() == h[480:]
    uncertainty = dict.fromkeys(("event_ns", "mapping_ns", "observer_ns", "representation_ns"), 0)
    result = assess(capture, *rest, uncertainties=uncertainty)
    assert result["budget_arithmetic_within_500us"] is True
    assert result["acquisition_registration"] == "BLOCKED"
    uncertainty["observer_ns"] = 500001
    assert not assess(capture, *rest, uncertainties=uncertainty)["budget_arithmetic_within_500us"]
    uncertainty["event_ns"] = -1
    with pytest.raises(ValueError):
        assess(capture, *rest, uncertainties=uncertainty)
    input_file, output_file = tmp_path / "context.json", tmp_path / "context.bin"
    input_file.write_text(json.dumps(capture["metadata"]))
    assert main(["context", str(input_file), str(output_file)]) == 0
    assert output_file.read_bytes() == h
    with pytest.raises(FileExistsError):
        main(["context", str(input_file), str(output_file)])


def test_frozen_steady_coverage_remains_offline():
    capture, raw, source, wire, context = fixture(2000)
    assert assess(capture, raw, source, wire, context)["hook_coverage"] == "BLOCKED"
    context.update(readiness_ns=0, steady_start_ns=5_026_000_000, steady_end_ns=65_053_000_000)
    result = assess(capture, raw, source, wire, context)
    assert result["hook_coverage"] == "PASS"
    assert result["steady_matched_count"] == 1820
    assert result["gps_qualification"] == "BLOCKED"
    context["steady_start_ns"] = 1
    with pytest.raises(ValueError):
        assess(capture, raw, source, wire, context)


def test_uncertainty_overflow_and_stale_build():
    capture, *rest = fixture()
    values = {"event_ns": (1 << 63) - 1, "mapping_ns": 1, "observer_ns": 0, "representation_ns": 0}
    with pytest.raises(ValueError):
        assess(capture, *rest, uncertainties=values)
    capture["metadata"]["binary_sha256"] = "cd" * 32
    assert "capture_context" in assess(capture, *rest)["failures"]


def test_patch_scope_and_compile_gate():
    from pathlib import Path

    patch = (
        Path(__file__).resolve().parents[2]
        / "integrations/px4/patches/0001-gps-acquisition-observer.patch"
    ).read_text()
    paths = [
        line.split(" b/", 1)[1] for line in patch.splitlines() if line.startswith("diff --git ")
    ]
    assert len(paths) == 5
    assert all(p.startswith("src/modules/simulation/gz_bridge/") for p in paths)
    assert 'option(AEROTWIN_GPS_OBSERVER "Validation-only GPS acquisition observer" OFF)' in patch
    bridge = patch.split("diff --git a/src/modules/simulation/gz_bridge/GZBridge.cpp", 1)[1].split(
        "diff --git", 1
    )[0]
    assert not any(
        line.startswith("-") and not line.startswith("---") for line in bridge.splitlines()
    )
    added = "\n".join(
        line[1:]
        for line in bridge.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    assert "hrt_absolute_time" not in added and "addGpsNoise(" not in added
    assert "published ? _sensor_gps_pub.get_instance() : -1" in added


@pytest.mark.parametrize("complete", [True, False])
def test_external_fifo_sink_retains_partial_evidence(tmp_path, complete):
    import os
    import time
    from concurrent.futures import ThreadPoolExecutor

    from aerotwin_experiments.m3_gps_hook import sink

    fifo, output = tmp_path / "owned.fifo", tmp_path / "capture.bin"
    blob = encode(fixture()[0])
    if not complete:
        blob = blob[:-1]
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(sink, fifo, output, 2)
        deadline = time.monotonic() + 2
        while not fifo.exists() and time.monotonic() < deadline:
            time.sleep(0.001)
        fd = os.open(fifo, os.O_WRONLY)
        try:
            assert os.write(fd, blob) == len(blob)
        finally:
            os.close(fd)
        if complete:
            future.result(timeout=3)
        else:
            with pytest.raises(ValueError):
                future.result(timeout=3)
    assert output.read_bytes() == blob
    assert not fifo.exists()
