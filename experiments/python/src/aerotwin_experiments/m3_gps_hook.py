"""Validation-only GPS hook binary ingestion and offline association; no live launcher."""

import argparse
import hashlib
import json
import os
import selectors
import struct
import time
import uuid
import zlib
from collections import Counter
from pathlib import Path

from .m3_clock import microseconds_to_nanoseconds
from .m3_gps_qualification import compare, payload, protocol, unsigned, validate_context

REVISION = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
DIGESTS = ("patch_sha256", "binary_sha256", "base_sha256", "config_sha256", "gazebo_sha256")
METADATA = {"run_id", "epoch", "generation", "sensor_topic", "frame_id", "upstream", *DIGESTS}
COUNTERS = (
    "callbacks",
    "enqueued",
    "drained",
    "overflow",
    "invalid",
    "write_errors",
    "high_water",
    "shutdown_failed",
)


def integer(value, maximum=(1 << 63) - 1):
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError("Invalid checked integer")
    return value


def header(metadata):
    """Encode exactly the frozen 512-byte context; no runtime configuration is verified here."""
    if set(metadata) != METADATA or metadata["upstream"] != REVISION:
        raise ValueError("Unexpected context schema or upstream revision")
    data = bytearray(480)
    data[:8] = b"ATGPSH1\0"
    for name, offset in (("run_id", 8), ("epoch", 24)):
        value = uuid.UUID(metadata[name])
        if not value.int or str(value) != metadata[name]:
            raise ValueError("Canonical nonzero UUID required")
        data[offset : offset + 16] = value.bytes
    generation = unsigned(metadata["generation"], 64)
    if not generation:
        raise ValueError("Positive generation required")
    struct.pack_into("<Q", data, 40, generation)
    for name, offset, size in (("sensor_topic", 48, 128), ("frame_id", 176, 64)):
        value = metadata[name].encode("ascii")
        if not value or len(value) >= size or any(v < 32 or v > 126 for v in value):
            raise ValueError("Invalid bounded sensor identity")
        data[offset : offset + len(value)] = value
    data[240:260] = bytes.fromhex(REVISION)
    for index, name in enumerate(DIGESTS):
        value = metadata[name]
        if not isinstance(value, str) or len(value) != 64 or value.lower() != value:
            raise ValueError("Canonical SHA-256 required")
        digest = bytes.fromhex(value)
        if len(digest) != 32 or not any(digest):
            raise ValueError("Nonzero SHA-256 required")
        data[260 + index * 32 : 292 + index * 32] = digest
    return bytes(data) + hashlib.sha256(data).digest()


def decode(blob):
    """Strict framing, context digest, CRC and reserved-bit checks; loss remains explicit."""
    blob = bytes(blob)
    if len(blob) < 640 or len(blob) > 64 * 1024 * 1024 or (len(blob) - 640) % 192:
        raise ValueError("Truncated capture or missing footer")
    h, footer = blob[:512], blob[-128:]
    if h[:8] != b"ATGPSH1\0" or hashlib.sha256(h[:480]).digest() != h[480:]:
        raise ValueError("Invalid context digest")
    metadata = {
        "run_id": str(uuid.UUID(bytes=h[8:24])),
        "epoch": str(uuid.UUID(bytes=h[24:40])),
        "generation": struct.unpack_from("<Q", h, 40)[0],
        "upstream": h[240:260].hex(),
    }
    for name, start, size in (("sensor_topic", 48, 128), ("frame_id", 176, 64)):
        metadata[name] = h[start : start + size].split(b"\0", 1)[0].decode("ascii")
    metadata.update({name: h[260 + i * 32 : 292 + i * 32].hex() for i, name in enumerate(DIGESTS)})
    if header(metadata) != h:
        raise ValueError("Noncanonical header")
    if footer[:8] != b"ATGPSF1\0" or footer[8:40] != h[480:] or any(footer[124:]):
        raise ValueError("Invalid footer context")
    records_blob = blob[512:-128]
    if (
        zlib.crc32(footer[:104] + bytes(4) + footer[108:], zlib.crc32(records_blob))
        != struct.unpack_from("<I", footer, 104)[0]
    ):
        raise ValueError("Capture CRC mismatch")
    records = []
    for offset in range(0, len(records_blob), 192):
        r = records_blob[offset : offset + 192]
        if r[:4] != b"EVT1" or any(r[4:8]) or r[8:40] != h[480:] or any(r[157:160] + r[184:]):
            raise ValueError("Invalid record framing or context")
        if r[154] not in (1, 2, 3) or r[155] > 1 or r[156] > 1:
            raise ValueError("Invalid record status")
        values = struct.unpack_from("<QQQqQQQIi", r, 40)
        names = (
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
        records.append(
            dict(zip(names, values, strict=True))
            | {
                "payload_hex": r[104:154].hex(),
                "status": r[154],
                "publish_ok": bool(r[155]),
                "vel_ned_valid": bool(r[156]),
                "host_brackets_ns": list(struct.unpack_from("<3Q", r, 160)),
            }
        )
    counts = dict(zip(COUNTERS, struct.unpack_from("<8Q", footer, 40), strict=True))
    return {
        "metadata": metadata,
        "records": records,
        "counters": counts,
        "overhead_ns": dict(
            zip(("queue_max", "callback_max"), struct.unpack_from("<2Q", footer, 108), strict=True)
        ),
    }


def payload_row(record):
    values = struct.unpack("<4d3fIBB", bytes.fromhex(record["payload_hex"]))
    names = (
        "latitude_deg",
        "longitude_deg",
        "altitude_msl_m",
        "altitude_ellipsoid_m",
        "vel_n_m_s",
        "vel_e_m_s",
        "vel_d_m_s",
        "device_id",
        "fix_type",
        "satellites_used",
    )
    return dict(zip(names, values, strict=True)) | {"vel_ned_valid": record["vel_ned_valid"]}


def assess(capture, raw, source, wire, context, uncertainties=None):
    """Evaluate trusted offline records. Synthetic data never qualify acquisition timing."""
    validate_context(context)
    m, records, counts = (capture[k] for k in ("metadata", "records", "counters"))
    header(m)
    failures = []
    if set(counts) != set(COUNTERS):
        raise ValueError("Invalid capture counter schema")
    for value in counts.values():
        unsigned(value, 64)
    if counts["high_water"] > 8192:
        failures.append("invalid_high_water")
    if any(m[k] != context[k] for k in METADATA):
        failures.append("capture_context")
    if (
        counts["enqueued"] != len(records)
        or counts["drained"] != len(records)
        or counts["callbacks"] != len(records)
    ):
        failures.append("missing_capture_records")
    if any(counts[k] for k in ("overflow", "invalid", "write_errors", "shutdown_failed")):
        failures.append("capture_loss")
    keys, events, times, previous = [], [], [], None
    for index, row in enumerate(records):
        try:
            if row["callback_seq"] != index or row["record_seq"] != index:
                raise ValueError("observation_order")
            event, acquisition = unsigned(row["event_seq"], 64), integer(row["acquisition_ns"])
            if previous is not None and (event != previous[0] + 1 or acquisition <= previous[1]):
                failures.append("event_order_or_gap")
            previous = (event, acquisition)
            events.append(event)
            if row["status"] == 1:
                if (
                    index != 0
                    or row["publish_ok"]
                    or row["device_id"]
                    or any(bytes.fromhex(row["payload_hex"]))
                ):
                    raise ValueError("invalid_initialization")
                continue
            if (
                row["status"] != 2
                or not row["publish_ok"]
                or row["receiver_instance"] != 0
                or row["device_id"] != context["device_id"]
            ):
                raise ValueError("invalid_publication_or_receiver")
            key = payload(payload_row(row))
            if key.hex() != row["payload_hex"] or payload_row(row)["device_id"] != row["device_id"]:
                raise ValueError("payload_representation")
            assigned = microseconds_to_nanoseconds(row["assigned_us"])
            if (
                row["timestamp_us"] != row["assigned_us"]
                or row["timestamp_sample_us"] != row["assigned_us"]
            ):
                raise ValueError("assigned_time_changed")
            if times and assigned <= times[-1][1]:
                raise ValueError("assigned_time_order")
            if not acquisition or not assigned:
                raise ValueError("zero_timestamp")
            times.append((acquisition, assigned, assigned - acquisition))
            keys.append(key)
        except (ValueError, KeyError, TypeError, OverflowError, struct.error) as exc:
            failures.append(str(exc))
    if len(events) != len(set(events)):
        failures.append("duplicate_event")
    stages = [keys]
    for side, rows in (("raw", raw), ("source", source), ("wire", wire)):
        stage = []
        last = None
        for row in rows:
            try:
                if any(
                    type(row[k]) is not type(context[k]) or row[k] != context[k]
                    for k in ("run_id", "epoch", "generation", "device_id")
                ):
                    raise ValueError("stale_record")
                if row["receiver_instance"] != 0 or type(row["receiver_instance"]) is not int:
                    raise ValueError("receiver_mismatch")
                if row["publisher"] != context[side + "_publisher"] or row["clock"] != "px4_hrt_us":
                    raise ValueError("publisher_or_clock")
                expected = {
                    "raw": "sensor_gps",
                    "source": "vehicle_gps_position",
                    "wire": "/fmu/out/vehicle_gps_position",
                }[side]
                if row["topic"] != expected:
                    raise ValueError("topic_mismatch")
                stamp = microseconds_to_nanoseconds(row["timestamp_sample_us"])
                microseconds_to_nanoseconds(row["timestamp_us"])
                if last is not None and stamp <= last:
                    raise ValueError("source_order")
                last = stamp
                stage.append(payload(row))
            except (ValueError, KeyError, TypeError, OverflowError, struct.error) as exc:
                failures.append(side + ":" + str(exc))
        stages.append(stage)
    if any(len(stage) != len(set(stage)) for stage in stages):
        failures.append("ambiguous_payload")
    if any(Counter(stage) != Counter(keys) for stage in stages[1:]):
        failures.append("unmatched_payload")
    # Identity is established before checking each downstream timestamp.
    if not failures:
        gps_records = [r for r in records if r["status"] == 2]
        hook = {bytes.fromhex(r["payload_hex"]): r for r in gps_records}
        for rows in (raw, source, wire):
            for row in rows:
                original = hook[payload(row)]
                if any(row[k] != original[k] for k in ("timestamp_us", "timestamp_sample_us")):
                    failures.append("timestamp_preservation")
    preservation = compare(source, wire, context)
    if preservation["timestamp_preservation"] != "PASS":
        failures.append("selected_source_wire_preservation")
    uncertainties = (
        dict.fromkeys(("event_ns", "mapping_ns", "observer_ns", "representation_ns"))
        if uncertainties is None
        else uncertainties
    )
    if set(uncertainties) != {"event_ns", "mapping_ns", "observer_ns", "representation_ns"}:
        raise ValueError("Unknown uncertainty component")
    for value in uncertainties.values():
        if value is not None:
            integer(value)
    observed = max((abs(t[2]) for t in times), default=None)
    total = None
    if observed is not None and all(v is not None for v in uncertainties.values()):
        total = integer(observed + sum(uncertainties.values()))
    window = []
    if "steady_start_ns" in context and "steady_end_ns" in context and "readiness_ns" in context:
        start, end, ready = (
            integer(context[k]) for k in ("steady_start_ns", "steady_end_ns", "readiness_ns")
        )
        if start < ready + 5_000_000_000 or end < start:
            raise ValueError("Invalid frozen steady interval")
        window = [t for t in times if start <= t[0] <= end]
    acquisition_span = window[-1][0] - window[0][0] if window else 0
    assigned_span = window[-1][1] - window[0][1] if window else 0
    gap = max((b[0] - a[0] for a, b in zip(window, window[1:])), default=0)
    coverage = (
        not failures
        and len(window) >= 1800
        and acquisition_span >= 60_000_000_000
        and assigned_span >= 60_000_000_000
        and gap <= 66_666_668
        and preservation["observation_coverage"] == "PASS"
    )
    return {
        "hook_coverage": "PASS" if coverage else "BLOCKED",
        "steady_matched_count": len(window) if not failures else 0,
        "acquisition_span_ns": acquisition_span,
        "assigned_span_ns": assigned_span,
        "maximum_acquisition_gap_ns": gap,
        "offline_association": "PASS" if not failures and keys else "FAIL",
        "failures": sorted(set(failures)),
        "matched_count": len(keys) if not failures else 0,
        "registration_error_ns": {
            "minimum": min((t[2] for t in times), default=None),
            "maximum": max((t[2] for t in times), default=None),
            "absolute_maximum": observed,
        },
        "uncertainties_ns": uncertainties,
        "conditional_total_ns": total,
        "budget_arithmetic_within_500us": total <= 500_000 if total is not None else None,
        "source_wire": preservation,
        "frozen_policy": protocol(),
        "acquisition_registration": "BLOCKED",
        "gps_qualification": "BLOCKED",
        "m34": "BLOCKED",
        "live_attempts_consumed": 0,
        "limitation": "Offline associations and supplied uncertainty arithmetic are not live evidence or a worst-case guarantee.",
    }


def sink(fifo, output, timeout_s=180):
    """Owned external FIFO sink. Bounded wall time/bytes; preserves incomplete evidence."""
    if type(timeout_s) is not int or not 1 <= timeout_s <= 180:
        raise ValueError("Capture timeout must be 1..180 wall seconds")
    fifo, output = Path(fifo), Path(output)
    os.mkfifo(fifo, 0o600)  # Exclusive: never overwrite an existing path.
    fd = None
    try:
        fd = os.open(fifo, os.O_RDONLY | os.O_NONBLOCK)
        with output.open("xb") as stream, selectors.DefaultSelector() as selector:
            selector.register(fd, selectors.EVENT_READ)
            deadline, count = time.monotonic() + timeout_s, 0
            while time.monotonic() < deadline:
                # Read after the bounded wait even without a readiness notification:
                # kqueue need not repeat an EOF notification after the final data read.
                selector.select(min(0.1, max(0, deadline - time.monotonic())))
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    if count:
                        break
                    time.sleep(0.01)
                    continue
                count += len(data)
                if count > 64 * 1024 * 1024:
                    raise ValueError("Capture size limit")
                stream.write(data)
            else:
                raise TimeoutError("Capture wall-time limit; partial evidence retained")
        decode(output.read_bytes())  # Missing footer/CRC is a failure, never a clean capture.
    finally:
        if fd is not None:
            os.close(fd)
        fifo.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("context")
    p.add_argument("input", type=Path)
    p.add_argument("output", type=Path)
    p = sub.add_parser("decode")
    p.add_argument("input", type=Path)
    p.add_argument("output", type=Path)
    p = sub.add_parser("assess")
    p.add_argument(
        "input", type=Path, help="JSON containing capture path, raw/source/wire arrays and context"
    )
    p.add_argument("output", type=Path)
    p = sub.add_parser("sink")
    p.add_argument("fifo", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--timeout", type=int, default=180)
    args = parser.parse_args(argv)
    if args.command == "sink":
        sink(args.fifo, args.output, args.timeout)
        return 0
    if args.command == "context":
        result = header(json.loads(args.input.read_text()))
    elif args.command == "decode":
        result = decode(args.input.read_bytes())
    else:
        values = json.loads(args.input.read_text())
        result = assess(
            decode(Path(values["capture"]).read_bytes()),
            values["raw"],
            values["source"],
            values["wire"],
            values["context"],
            values.get("uncertainties"),
        )
    with args.output.open("xb") as stream:
        stream.write(
            result
            if isinstance(result, bytes)
            else (json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
        )
    return 2 if args.command == "assess" else 0  # Qualification deliberately remains blocked.


if __name__ == "__main__":
    raise SystemExit(main())
