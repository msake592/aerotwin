"""Offline SensorGps preservation checks, never physical acquisition qualification."""

import argparse
import hashlib
import json
import math
import struct
from collections import Counter
from pathlib import Path

from .m3_clock import microseconds_to_nanoseconds

SOURCE_TOPIC = "vehicle_gps_position"
WIRE_TOPIC = "/fmu/out/vehicle_gps_position"
PARAMETERS = {"UXRCE_DDS_SYNCT": 0, "UXRCE_DDS_SYNCC": 0, "SENS_GPS_MASK": 0, "SENS_GPS_PRIME": 0}
RATE_HZ = 30
DURATION_NS = 60_000_000_000
MINIMUM_OBSERVATIONS = RATE_HZ * (DURATION_NS // 1_000_000_000)
MAXIMUM_GAP_NS = 2 * ((1_000_000_000 + RATE_HZ - 1) // RATE_HZ)


def protocol():
    """Acceptance policy fixed before any F.1 live result, not a runtime launcher."""
    return {
        "schema": 1,
        "parameters": dict(PARAMETERS),
        "source_topic": SOURCE_TOPIC,
        "wire_topic": WIRE_TOPIC,
        "receiver_instance": 0,
        "configured_sensor_rate_hz": RATE_HZ,
        "required_sessions": 2,
        "maximum_live_attempts": 2,
        "steady_duration_ns": DURATION_NS,
        "minimum_source_wire_and_matched_count": MINIMUM_OBSERVATIONS,
        "maximum_gap_ns": MAXIMUM_GAP_NS,
        "startup_exclusion_ns_after_verified_configuration": 5_000_000_000,
        "maximum_attempt_wall_s": 180,
        "maximum_cleanup_wall_s": 30,
        "registration_budget_ns": 500_000,
        "pairwise_budget_ns": 1_000_000,
        "required_timestamp_difference_ns": 0,
        "maximum_ambiguous_invalid_unmatched_or_ordering": 0,
        "acquisition_reference_required": True,
        "motion_coverage_required": ["stationary", "bounded_nonstationary"],
        "independent_reference_available": False,
        "live_authorization_gate": "BLOCKED: no independent event-to-assigned-time association",
    }


def unsigned(value, bits):
    if type(value) is not int or not 0 <= value < (1 << bits):
        raise ValueError("Invalid unsigned field")
    return value


def payload(row):
    """Canonical native precisions, excluding both timestamp fields and receipt time."""
    coordinates = [
        row[name]
        for name in ("latitude_deg", "longitude_deg", "altitude_msl_m", "altitude_ellipsoid_m")
    ]
    velocity = [row[name] for name in ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s")]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in coordinates + velocity):
        raise ValueError("Invalid GPS payload")
    if not -90 <= coordinates[0] <= 90 or not -180 <= coordinates[1] <= 180:
        raise ValueError("Invalid geodetic coordinates")
    fix = unsigned(row["fix_type"], 8)
    if fix != 3 or row["vel_ned_valid"] is not True:
        raise ValueError("Invalid GPS fix/velocity")
    if unsigned(row["device_id"], 32) == 0:
        raise ValueError("Blended or missing device identity")
    # Native float32 input must remain finite after packing as well.
    packed = struct.pack(
        "<4d3fIBB",
        *coordinates,
        *velocity,
        row["device_id"],
        fix,
        unsigned(row["satellites_used"], 8),
    )
    if not all(math.isfinite(v) for v in struct.unpack("<3f", packed[32:44])):
        raise ValueError("Nonfinite native float32")
    return packed


def validate_context(context):
    if not isinstance(context.get("epoch"), str) or not context["epoch"]:
        raise ValueError("Explicit epoch required")
    if type(context.get("generation")) is not int or context["generation"] < 1:
        raise ValueError("Explicit mapping generation required")
    for name in ("source_publisher", "wire_publisher"):
        if not isinstance(context.get(name), str) or not context[name]:
            raise ValueError("Explicit publisher identities required")
    if (
        context.get("receiver_instances") != [0]
        or type(context["receiver_instances"][0]) is not int
        or type(context.get("selected_receiver")) is not int
        or context["selected_receiver"] != 0
    ):
        raise ValueError("Verified single receiver instance zero required")
    settings = context.get("parameters", {})
    if any(type(settings.get(k)) is not int or settings[k] != v for k, v in PARAMETERS.items()):
        raise ValueError("Verified no-adjustment/single-receiver configuration required")
    if unsigned(context["device_id"], 32) == 0:
        raise ValueError("Single-receiver device identity required")


def compare(source, wire, context):
    """Compare trusted ingress records. Cannot reconstruct missing session provenance.

    Per-record epoch/generation/publisher metadata must originate at capture time.
    Never relabel queued records with the current context before calling this function.
    """
    validate_context(context)
    indexed, invalid, ordering, spans, gaps = [], [], [], [], []
    for side, rows, topic in (("source", source, SOURCE_TOPIC), ("wire", wire, WIRE_TOPIC)):
        valid, errors, violations, times, previous = [], 0, 0, [], None
        for row in rows:
            try:
                for key in ("epoch", "generation", "device_id"):
                    if type(row[key]) is not type(context[key]) or row[key] != context[key]:
                        raise ValueError("Stale or mismatched provenance")
                if row["publisher"] != context[side + "_publisher"] or row["topic"] != topic:
                    raise ValueError("Publisher or topic changed")
                if type(row["receiver_instance"]) is not int or row["receiver_instance"] != 0:
                    raise ValueError("Unsupported receiver")
                if row["clock"] != "px4_hrt_us":
                    raise ValueError("Unsupported clock")
                publication = microseconds_to_nanoseconds(row["timestamp_us"])
                acquisition = microseconds_to_nanoseconds(row["timestamp_sample_us"])
                if acquisition > publication:
                    raise ValueError("Acquisition after publication")
                key = payload(row)
                if previous is not None and (
                    publication <= previous[0] or acquisition <= previous[1]
                ):
                    violations += 1
                previous = publication, acquisition
                valid.append((key, publication, acquisition))
                times.append(acquisition)
            except (KeyError, TypeError, ValueError, OverflowError, struct.error):
                errors += 1
        indexed.append(valid)
        invalid.append(errors)
        ordering.append(violations)
        spans.append(max(times) - min(times) if times else 0)
        gaps.append(max((b - a for a, b in zip(times, times[1:])), default=0))
    counts = [Counter(row[0] for row in rows) for rows in indexed]
    lookup = {row[0]: row for row in indexed[0]}
    pairs = []
    for key, publication, acquisition in indexed[1]:
        if counts[0][key] != 1 or counts[1][key] != 1:
            continue
        original = lookup[key]
        pairs.append(
            {
                "fingerprint": hashlib.sha256(key).hexdigest(),
                "source_timestamp_ns": original[1],
                "source_timestamp_sample_ns": original[2],
                "wire_timestamp_ns": publication,
                "wire_timestamp_sample_ns": acquisition,
                "publication_difference_ns": publication - original[1],
                "acquisition_difference_ns": acquisition - original[2],
            }
        )
    ambiguous = [
        sum(n for key, n in c.items() if n > 1 or counts[1 - i][key] > 1)
        for i, c in enumerate(counts)
    ]
    unmatched = [len(rows) - len(pairs) for rows in (source, wire)]
    differing = sum(
        p["publication_difference_ns"] != 0 or p["acquisition_difference_ns"] != 0 for p in pairs
    )
    issues = sum(invalid + ordering + ambiguous + unmatched) + differing
    matched_span = max((p["source_timestamp_sample_ns"] for p in pairs), default=0) - min(
        (p["source_timestamp_sample_ns"] for p in pairs), default=0
    )
    coverage = (
        min(len(source), len(wire), len(pairs)) >= MINIMUM_OBSERVATIONS
        and min(spans + [matched_span]) >= DURATION_NS
        and max(gaps) <= MAXIMUM_GAP_NS
    )
    return {
        "evidence_scope": "provided records only; input provenance must be independently verified",
        "source_count": len(source),
        "wire_count": len(wire),
        "matched": len(pairs),
        "invalid_source_wire": invalid,
        "ambiguous_source_wire": ambiguous,
        "unmatched_source_wire": unmatched,
        "ordering_source_wire": ordering,
        "source_wire_spans_ns": spans,
        "maximum_gaps_ns": gaps,
        "matched_span_ns": matched_span,
        "differing_pairs": differing,
        "pairs": pairs,
        "timestamp_preservation": ("FAIL" if issues else "PASS") if source and wire else "BLOCKED",
        "observation_coverage": "PASS" if coverage and not issues else "BLOCKED",
        "independent_acquisition_registration": "BLOCKED",
        "registration_uncertainty_ns": None,
        "timing_budget_compliance": "BLOCKED",
        "gps_qualification": "BLOCKED",
        "mapping_valid": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path, help="JSON containing context, source and wire arrays"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output file")
    result = {"protocol": protocol(), "live_attempts": 0, "gps_qualification": "BLOCKED"}
    if args.input:
        data = json.loads(args.input.read_text())
        result = compare(data["source"], data["wire"], data["context"])
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return 2  # Physical qualification remains blocked, even after exact preservation.


if __name__ == "__main__":
    raise SystemExit(main())
