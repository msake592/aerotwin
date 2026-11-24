"""Timestamp-independent VehicleOdometry payload matching for validation."""

import hashlib
import math
import struct
from collections import Counter

from .m3_clock import microseconds_to_nanoseconds


def canonical_payload(row):
    payload = []
    for name, length in (("position_m", 3), ("velocity_mps", 3), ("attitude_wxyz", 4)):
        vector = row[name]
        if len(vector) != length or not all(math.isfinite(value) for value in vector):
            raise ValueError("Missing or nonfinite payload")
        payload.extend(vector)
    frames = [row[name] for name in ("pose_frame", "velocity_frame", "reset_counter")]
    if any(type(value) is not int or not 0 <= value <= 255 for value in frames):
        raise ValueError("Invalid frame or reset metadata")
    if frames[0] not in (1, 2) or frames[1] not in (1, 2, 3):
        raise ValueError("Unsupported source frames")
    if not math.isclose(math.hypot(*row["attitude_wxyz"]), 1.0, abs_tol=1e-3):
        raise ValueError("Invalid orientation")
    # Float32 bytes preserve signed zero. Timestamp fields are deliberately excluded.
    return struct.pack("<10f3B", *payload, *frames)


def fingerprint(row):
    return hashlib.sha256(canonical_payload(row)).hexdigest()


def match(source, wire):
    indexed = []
    invalid = 0
    for rows in (source, wire):
        values = []
        for row in rows:
            try:
                key = (row.get("epoch"), row.get("generation"), fingerprint(row))
                microseconds_to_nanoseconds(row["timestamp_us"])
                microseconds_to_nanoseconds(row["timestamp_sample_us"])
                if row["timestamp_sample_us"] > row["timestamp_us"]:
                    raise ValueError("Acquisition after publication")
                values.append((key, row))
            except (KeyError, TypeError, ValueError, OverflowError, struct.error):
                invalid += 1
        indexed.append(values)
    counts = [Counter(key for key, _ in rows) for rows in indexed]
    lookup = dict(indexed[0])
    pairs = []
    for key, row in indexed[1]:
        if counts[0][key] != 1 or counts[1][key] != 1:
            continue
        original = lookup[key]
        if canonical_payload(original) != canonical_payload(row):
            continue
        pairs.append(
            {
                "fingerprint": key[2],
                "epoch": key[0],
                "generation": key[1],
                "source_timestamp_us": original["timestamp_us"],
                "source_timestamp_sample_us": original["timestamp_sample_us"],
                "wire_timestamp_sample_us": row["timestamp_sample_us"],
                "wire_timestamp_us": row["timestamp_us"],
                "publication_offset_ns": microseconds_to_nanoseconds(row["timestamp_us"])
                - microseconds_to_nanoseconds(original["timestamp_us"]),
                "acquisition_offset_ns": microseconds_to_nanoseconds(row["timestamp_sample_us"])
                - microseconds_to_nanoseconds(original["timestamp_sample_us"]),
            }
        )
    return {
        "matched": len(pairs),
        "unmatched_wire": len(wire) - len(pairs),
        "invalid": invalid,
        "pairs": pairs,
        "uncertainty_ns": None,
        "mapping_valid": False,
    }


def ulog_rows(path, metadata=None):
    from pyulog import ULog

    log = ULog(str(path), ["vehicle_odometry"])
    rows = []
    for dataset in log.data_list:
        if dataset.name != "vehicle_odometry" or dataset.multi_id != 0:
            continue
        data = dataset.data
        for index in range(len(data["timestamp"])):
            row = {
                name: int(data[name][index])
                for name in (
                    "timestamp",
                    "timestamp_sample",
                    "pose_frame",
                    "velocity_frame",
                    "reset_counter",
                )
            }
            row["timestamp_us"] = row.pop("timestamp")
            row["timestamp_sample_us"] = row.pop("timestamp_sample")
            for output, field, length in (
                ("position_m", "position", 3),
                ("velocity_mps", "velocity", 3),
                ("attitude_wxyz", "q", 4),
            ):
                values = [float(data[f"{field}[{n}]"][index]) for n in range(length)]
                row[output] = [value if math.isfinite(value) else None for value in values]
            rows.append(row)
    if metadata is not None:
        intervals = [b["timestamp_us"] - a["timestamp_us"] for a, b in zip(rows, rows[1:])]
        metadata.update(
            source_file=str(path),
            rows=len(rows),
            dropouts=len(log.dropouts),
            min_interval_us=min(intervals) if intervals else None,
            max_interval_us=max(intervals) if intervals else None,
        )
    return rows


def summarize(result):
    offsets = sorted(pair["publication_offset_ns"] for pair in result["pairs"])
    acquisition = sorted(pair["acquisition_offset_ns"] for pair in result["pairs"])

    def distribution(values):
        if not values:
            return None
        median = values[len(values) // 2]
        residuals = sorted(abs(value - median) for value in values)
        return {
            "min_ns": values[0],
            "max_ns": values[-1],
            "median_ns": median,
            "absolute_residual_p95_ns": residuals[(len(values) - 1) * 95 // 100],
            "absolute_residual_max_ns": residuals[-1],
        }

    ordered = sorted(result["pairs"], key=lambda pair: pair["source_timestamp_us"])
    source_span = (
        ordered[-1]["source_timestamp_us"] - ordered[0]["source_timestamp_us"] if ordered else 0
    )
    wire_span = ordered[-1]["wire_timestamp_us"] - ordered[0]["wire_timestamp_us"] if ordered else 0
    return {
        "source_span_us": source_span,
        "wire_span_us": wire_span,
        "rate_span_difference_us": wire_span - source_span,
        "arrival_ordering_violations": sum(
            b["wire_timestamp_us"] <= a["wire_timestamp_us"]
            for a, b in zip(result["pairs"], result["pairs"][1:])
        ),
        "wire_ordering_violations": sum(
            b["wire_timestamp_us"] <= a["wire_timestamp_us"] for a, b in zip(ordered, ordered[1:])
        ),
        "matched": result["matched"],
        "unmatched_wire": result["unmatched_wire"],
        "invalid": result["invalid"],
        "publication": distribution(offsets),
        "acquisition": distribution(acquisition),
        "publication_acquisition_offset_difference_ns": [
            pair["publication_offset_ns"] - pair["acquisition_offset_ns"]
            for pair in result["pairs"]
        ],
        "uncertainty_ns": None,
        "mapping_valid": False,
    }


def main():
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ulog-directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = []
    metadata = []
    for path in sorted(args.ulog_directory.rglob("*.ulg")):
        record = {}
        rows.extend(ulog_rows(path, record))
        metadata.append(record)
    args.output.with_suffix(".metadata.json").write_text(json.dumps(metadata))
    args.output.write_text(json.dumps(rows, allow_nan=False))
    print(json.dumps({"source_rows": len(rows)}))


if __name__ == "__main__":
    main()
