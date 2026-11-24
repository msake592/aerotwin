"""Bounded clock evidence collection and retained reconnection normalization."""

import argparse
import json
import re
import signal
from pathlib import Path

from .m3_clock import Mapping, Observation, correlate, normalize_reconnection
from .m3_docker import Session, samples, save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--retained", type=Path, required=True)
    args = parser.parse_args()

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", args.image):
        parser.error("Use an immutable retained image ID")
    if args.evidence.exists():
        parser.error("Use a new evidence directory")
    args.evidence.mkdir(parents=True)
    raw = json.loads((args.retained / "summary.json").read_text())
    save(args.evidence / "derived-reconnection.json", [normalize_reconnection(row) for row in raw])
    compose = Path(__file__).resolve().parents[4] / "simulation/docker/compose.yaml"
    mapping = Mapping("uninitialized")
    reports = []
    for number in (1, 2):
        directory = args.evidence / f"epoch-{number}"
        directory.mkdir()
        session = Session(compose, args.image, directory)
        mapping.restart(session.identity)
        result = session.run(clock_experiments=False)
        if result.get("interrupted"):
            reports.append(
                {"session": result["identity"], "cleanup": result["cleanup"], "interrupted": True}
            )
            break
        values = samples(directory / "samples.jsonl")
        observations = [
            Observation(
                session.identity,
                mapping.generation,
                json.dumps(row["publisher_gid"]),
                f"receipt-{index}",
                row["timestamp_sample_us"],
                "xrce-adjusted-us",
                row["receipt_monotonic_ns"],
            )
            for index, row in enumerate(values)
        ]
        # The uORB CLI prints rounded vectors and exposes no wire message ID.
        # Receipt IDs identify capture rows only and cannot identify source measurements.
        correlation = correlate([], observations)
        save(directory / "correlation.json", correlation)
        save(
            directory / "epoch.json",
            {
                "epoch": mapping.epoch,
                "mapping_generation": mapping.generation,
                "mapping_state": mapping.state.value,
                "mapping_valid": False,
                "source_observations": result.get("clocks", {}).get("normal"),
                "identity_limitation": "No independently verified unique identity connects wire rows to uORB CLI output",
            },
        )
        reports.append(
            {
                "session": result["identity"],
                "transport": result["transport"],
                "cleanup": result["cleanup"],
                "correlation": correlation,
            }
        )
    save(args.evidence / "summary.json", reports)
    complete = len(reports) == 2 and not any(row.get("interrupted") for row in reports)
    if complete:
        (args.evidence / "complete").touch()
    return 0 if complete and all(row["cleanup"] == "PASS" for row in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
