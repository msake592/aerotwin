"""Deterministic timing experiments invoking the production C++ M2 estimator."""

import argparse
import csv
import json
import statistics
import subprocess
from pathlib import Path

ERRORS_NS = (
    0,
    1000,
    3000,
    100_000,
    500_000,
    1_000_000,
    5_000_000,
    10_000_000,
    50_000_000,
    100_000_000,
)
DRIFT_PPM = (0, 1, 10, 25, 100, 125, 250, 1000, 12500, 25000)
MOTIONS = ("stationary", "cv", "accelerating")
SEEDS = tuple(range(42, 62))


def configurations():
    for motion in MOTIONS:
        for seed in SEEDS:
            yield {"motion": motion, "mode": "baseline", "error": 0, "seed": seed}
            for mode in ("common", "stream", "acquisition", "jitter", "latency"):
                for amplitude in ERRORS_NS:
                    signs = (
                        (-1, 1)
                        if amplitude and mode in ("common", "stream", "acquisition")
                        else (1,)
                    )
                    for sign in signs:
                        yield {
                            "motion": motion,
                            "mode": mode,
                            "error": sign * amplitude,
                            "seed": seed,
                        }
            for rate in DRIFT_PPM:
                for sign in (-1, 1) if rate else (1,):
                    yield {"motion": motion, "mode": "drift", "error": rate * sign, "seed": seed}
            yield {"motion": motion, "mode": "oos", "error": 0, "seed": seed}


def execute(executable, configs):
    payload = "".join(f"{c['motion']} {c['mode']} {c['error']} {c['seed']}\n" for c in configs)
    completed = subprocess.run(
        [str(executable)], input=payload, capture_output=True, text=True, check=True, timeout=120
    )
    metrics = [json.loads(line) for line in completed.stdout.splitlines()]
    if len(metrics) != len(configs):
        raise ValueError("C++ experiment response count mismatch")
    rows = [config | values for config, values in zip(configs, metrics)]
    baseline = {(r["motion"], r["seed"]): r for r in rows if r["mode"] == "baseline"}
    for row in rows:
        reference = baseline.get((row["motion"], row["seed"]))
        if reference:
            for field in (
                "position_rmse",
                "velocity_rmse",
                "max_position_error",
                "mean_nis",
                "mean_nees",
            ):
                row["delta_" + field] = row[field] - reference[field]
        row["error_unit"] = "ppm" if row["mode"] == "drift" else "ns"
        row["drift_ppm"] = row["error"] if row["mode"] == "drift" else 0
        row["delivery_delay_ns"] = (
            row["error"]
            if row["mode"] == "latency"
            else (2_000_000_000 + row["error"] if row["mode"] == "oos" else 0)
        )
    return rows


def aggregate(rows):
    groups = {}
    for row in rows:
        key = (row["motion"], row["mode"], row["error"])
        groups.setdefault(key, []).append(row)
    result = []
    fields = (
        "position_rmse",
        "velocity_rmse",
        "max_position_error",
        "mean_nis",
        "mean_nees",
        "delta_position_rmse",
        "delta_velocity_rmse",
        "delta_max_position_error",
        "minimum_covariance_eigenvalue",
        "mean_position_trace",
        "mean_velocity_trace",
    )
    for (motion, mode, error), group in groups.items():
        record = {"motion": motion, "mode": mode, "error": error, "realizations": len(group)}
        for field in fields:
            record[field + "_mean"] = statistics.fmean(row[field] for row in group)
            record[field + "_max_abs"] = max(abs(row[field]) for row in group)
        record["accepted"] = sum(row["accepted"] for row in group)
        record["rejected"] = sum(row["rejected"] for row in group)
        record["ordering_violations"] = sum(row["ordering_violations"] for row in group)
        result.append(record)
    return result


def plot(results, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for metric, units in (
        ("position_rmse", "m"),
        ("velocity_rmse", "m/s"),
        ("mean_nis", "dof 3"),
        ("mean_nees", "dof 6"),
    ):
        figure, axes = plt.subplots(1, 3, figsize=(14, 4), constrained_layout=True)
        for axis, motion in zip(axes, MOTIONS):
            for mode in ("stream", "acquisition", "jitter", "latency"):
                values = sorted(
                    (r for r in results if r["motion"] == motion and r["mode"] == mode),
                    key=lambda r: r["error"],
                )
                axis.plot(
                    [r["error"] / 1e6 for r in values],
                    [r[metric + "_mean"] for r in values],
                    marker=".",
                    label={
                        "stream": "stream offset",
                        "acquisition": "acquisition shift",
                        "jitter": "jitter bound",
                        "latency": "delivery delay",
                    }[mode],
                )
            axis.set_xscale("symlog", linthresh=0.001)
            axis.set_xticks([-100, -1, -0.001, 0, 0.001, 1, 100])
            axis.set_xticklabels(["−100", "−1", "−0.001", "0", "0.001", "1", "100"], rotation=30)
            baseline = next(r for r in results if r["motion"] == motion and r["mode"] == "baseline")
            axis.axhline(
                baseline[metric + "_mean"],
                color="black",
                linestyle="--",
                linewidth=0.8,
                label="baseline",
            )
            axis.set_xlabel("Timing error / delivery delay (ms)")
            axis.set_ylabel(f"{metric} ({units})")
            axis.set_title(motion)
            axis.grid(alpha=0.3)
        axes[-1].legend()
        figure.savefig(output / f"{metric}.png", dpi=180)
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plots", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new artifact directory")
    args.output.mkdir(parents=True)
    configs = list(configurations())
    rows = execute(args.executable.resolve(), configs)
    summary = aggregate(rows)
    (args.output / "configurations.json").write_text(json.dumps(configs, indent=2))
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
    with (args.output / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    if args.plots:
        plot(summary, args.output)


if __name__ == "__main__":
    main()
