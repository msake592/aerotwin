"""Run offline Linux observer executable tests. No Gazebo simulation/PX4 is launched."""

import os
import selectors
import struct
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

from aerotwin_experiments.m3_gps_hook import REVISION, decode, header


def run(executable, root, mode, extra=None, stalled=False, broken=False):
    fifo = root / (mode + str(time.monotonic_ns()) + ".fifo")
    os.mkfifo(fifo)
    fd = os.open(fifo, os.O_RDONLY | os.O_NONBLOCK)
    env = os.environ | {"AEROTWIN_GPS_OBSERVER": "1", "AEROTWIN_GPS_FIFO": str(fifo)}
    env.update(extra or {})
    marker = root / "marker"
    output = bytearray()
    started = time.monotonic()
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(fd, selectors.EVENT_READ)
            process = subprocess.Popen([executable, mode, str(marker)], env=env)
            while process.poll() is None:
                if time.monotonic() - started > 8:
                    process.kill()
                    process.wait()
                    raise AssertionError("Bounded offline shutdown exceeded")
                if selector.select(0.01):
                    if stalled and len(output) >= 512:
                        time.sleep(0.01)
                        continue
                    chunk = os.read(fd, 4096)
                    output.extend(chunk)
                    if broken and len(output) >= 512:
                        selector.unregister(fd)
                        os.close(fd)
                        fd = None
                        broken = False
                    if mode == "overflow":
                        time.sleep(0.001)
            assert process.returncode == 0, (mode, process.returncode)
            if fd is not None and not stalled:
                while True:
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        break
                    output.extend(chunk)
    finally:
        if fd is not None:
            os.close(fd)
        fifo.unlink()
    return bytes(output), time.monotonic() - started


def main():
    executable = sys.argv[1]
    with tempfile.TemporaryDirectory(prefix="f3-offline-") as temporary:
        root = Path(temporary)
        m = {
            "run_id": str(uuid.UUID(int=1)),
            "epoch": str(uuid.UUID(int=2)),
            "generation": 1,
            "sensor_topic": "/test",
            "frame_id": "gps",
            "upstream": REVISION,
            **{k + "_sha256": "ab" * 32 for k in ("patch", "binary", "base", "config", "gazebo")},
        }
        context = root / "context.bin"
        context.write_bytes(header(m))
        os.environ["AEROTWIN_GPS_CONTEXT"] = str(context)
        checks = 0
        for mode in ("queue", "disabled"):
            subprocess.run([executable, mode, str(root / "marker")], check=True)
            checks += 1
        for override in (
            {"AEROTWIN_GPS_OBSERVER": "invalid"},
            {"AEROTWIN_GPS_CONTEXT": "/missing"},
            {"AEROTWIN_GPS_TIMING": "invalid"},
        ):
            data, _ = run(executable, root, "invalid", override)
            assert not data
            checks += 1
        invalid = root / "bad.bin"
        corrupt = bytearray(context.read_bytes())
        corrupt[480] ^= 1
        invalid.write_bytes(corrupt)
        data, _ = run(executable, root, "invalid", {"AEROTWIN_GPS_CONTEXT": str(invalid)})
        assert not data
        checks += 1
        data, _ = run(executable, root, "normal")
        capture = decode(data)
        assert capture["counters"]["callbacks"] == capture["counters"]["drained"] == 3
        assert capture["records"][0]["status"] == 1
        row = capture["records"][1]
        assert row["assigned_us"] == row["timestamp_us"] == row["timestamp_sample_us"] == 1033000
        assert bytes.fromhex(row["payload_hex"])[32:36] == struct.pack("<f", -0.0)
        assert row["event_seq"] == 1 and row["acquisition_ns"] == 1033000000
        checks += 1
        data, _ = run(executable, root, "normal", {"AEROTWIN_GPS_TIMING": "1"})
        timed = decode(data)
        assert timed["overhead_ns"]["queue_max"] > 0
        assert timed["overhead_ns"]["callback_max"] > 0
        assert all(r["host_brackets_ns"][0] > 0 for r in timed["records"])
        checks += 1
        data, _ = run(executable, root, "malformed")
        assert decode(data)["counters"]["invalid"] == 2
        checks += 1
        data, _ = run(executable, root, "overflow")
        capture = decode(data)
        assert capture["counters"]["overflow"] > 0
        assert capture["counters"]["callbacks"] == 50000
        assert (
            capture["counters"]["callbacks"]
            == capture["counters"]["enqueued"] + capture["counters"]["overflow"]
        )
        checks += 1
        for options in ({"stalled": True}, {"broken": True}):
            data, elapsed = run(executable, root, "overflow", **options)
            assert elapsed < 8
            try:
                decode(data)
            except ValueError:
                pass
            else:
                raise AssertionError("Incomplete capture accepted")
            checks += 1
        print(f"PASS: {checks} offline observer scenarios; no SITL or qualification attempts")


if __name__ == "__main__":
    main()
