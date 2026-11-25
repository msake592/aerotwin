"""Focused source-to-wire capture using the existing runtime and ULog parser."""

import argparse
import json
import os
import re
from pathlib import Path

from .m3_docker import Session, command, samples, save
from .m3_source_wire import match, summarize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", args.image) or args.evidence.exists():
        parser.error("Use an immutable image ID and a new evidence directory")
    root = args.evidence.resolve()
    root.mkdir(parents=True)
    os.environ["AEROTWIN_SOURCE_CAPTURE"] = "1"
    source_path = Path(__file__).resolve().parents[1]
    compose = Path(__file__).resolve().parents[4] / "simulation/docker/compose.yaml"
    reports = []
    for number in (1, 2):
        directory = root / f"session-{number}"
        directory.mkdir()
        session = Session(compose, args.image, directory)
        runtime = session.run(clock_experiments=False)
        save(directory / "runtime.json", runtime)
        if runtime["transport"] != "PASS" or runtime["cleanup"] != "PASS":
            raise RuntimeError("Source capture requires successful transport and cleanup")
        command(
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            "--entrypoint",
            "/work/venv/bin/python",
            "-v",
            f"{source_path}:/aerotwin:ro",
            "-v",
            f"{directory}:/evidence",
            "-e",
            "PYTHONPATH=/aerotwin",
            args.image,
            "-m",
            "aerotwin_experiments.m3_source_wire",
            "--ulog-directory",
            "/evidence/ulog",
            "--output",
            "/evidence/source.json",
            timeout=30,
        )
        source = json.loads((directory / "source.json").read_text())
        wire = samples(directory / "samples.jsonl")
        for observation in source + wire:
            observation.update(epoch=session.identity, generation=0)
        result = match(source, wire)
        save(directory / "matches.json", result)
        report = summarize(result)
        report.update(
            epoch=session.identity,
            source_rows=len(source),
            wire_rows=len(wire),
            publisher=runtime["observer"]["publisher_gid"],
        )
        save(directory / "measurement.json", report)
        reports.append(report)
    save(root / "summary.json", reports)
    (root / "complete").touch()
    return 0 if all(row["matched"] >= 3 and row["source_span_us"] > 0 for row in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
