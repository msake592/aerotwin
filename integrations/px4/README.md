# Validation-only PX4 GPS observation patch

This directory contains a narrow upstream patch and offline tests, not a vendored
PX4 tree or a production AeroTwin adapter. See the
[F.3 implementation report](../../docs/reports/m3-gps-hook-implementation.md) and
[F.4 execution gate](../../docs/guides/m3-gps-hook-qualification.md).

**Live execution is blocked by F.4 review finding R1:** unsubscribe does not
quiesce active or already-selected callbacks before observer state is freed.
The unchanged patch/snapshot are review artifacts. Do not run qualification
or repeat firmware builds without the required repair approval. See the
[final gate](../../docs/reports/m3-gps-acquisition-final-gate.md).
The build commands below document the historical F.3 reproduction procedure.

The patch applies only to PX4
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. Use an isolated copy of the retained
runtime with its existing build cache, identical absolute source/build paths,
no shared original volumes, no network, two CPUs and 6 GiB RAM. Do not apply it
to the original stopped container. Do not clean the cache or install packages.

Review and apply in the isolated checkout:

```sh
git rev-parse HEAD
git status --short
git submodule status --recursive
git apply --check --whitespace=error-all /evidence/0001-gps-acquisition-observer.patch
git apply /evidence/0001-gps-acquisition-observer.patch
export PATH=/work/venv/bin:$PATH
cmake -S . -B build/px4_sitl_default -DAEROTWIN_GPS_OBSERVER=ON
ninja -C build/px4_sitl_default -n modules__simulation__gz_bridge px4
```

Review the dry plan before compiling. Stop on a missing cache, mismatched pins,
package provenance, broad unrelated recompilation or required clean rebuild.
Preserve mtimes of byte-identical copied inputs; do not backdate modified code.
The historical F.3 targets, requiring separate build approval now, are:

```sh
cmake --build build/px4_sitl_default --target modules__simulation__gz_bridge --parallel 2
cmake --build build/px4_sitl_default --target px4 --parallel 2
```

Do not invoke `gz_x500`: that target launches simulation. The build option is
OFF by default. Even when compiled ON, observation remains disabled unless
`AEROTWIN_GPS_OBSERVER=1`. The review snapshot explicitly defaults it to 0.

## Context and wire format

`aerotwin_experiments.m3_gps_hook` uses only Python standard libraries and the
existing M3 clock/GPS utilities. Its `context`, `decode`, `assess` and `sink`
subcommands do not launch PX4, ROS or Gazebo. `assess` always exits 2 because
GPS qualification remains BLOCKED, even when offline associations pass.
Output paths are exclusive; existing evidence is never overwritten.

Context JSON has exactly: `run_id`, `epoch` (canonical nonzero UUIDs),
`generation` (positive uint64), `sensor_topic` (ASCII, <128 bytes), `frame_id`
(ASCII, <64 bytes), `upstream` (the pinned SHA), and lowercase nonzero 64-character
`patch_sha256`, `binary_sha256`, `base_sha256`, `config_sha256`, `gazebo_sha256`.
The Gazebo manifest digest covers installed package/library identities and source
provenance; the configuration manifest covers model/world, seeds, resource and
clock/receiver settings. Preserve both manifests, not just their digests.
These are evidence bindings, not proof that requested settings were applied.

The 512-byte header is little endian: magic `ATGPSH1\0` [0:8], run UUID [8:24],
epoch UUID [24:40], generation [40:48], zero-padded topic [48:176], frame
[176:240], 20-byte upstream SHA [240:260], five 32-byte digests in the order
listed above [260:420], zero reserved bytes [420:480], SHA-256 of bytes [0:480]
at [480:512]. C++ and Python verify the header digest. This is integrity checking,
not authentication against a malicious producer.

Each 192-byte event contains magic `EVT1` [0:4], zero flags [4:8], context digest
[8:40], seven 64-bit fields at [40:96]: callback ordinal, queue record ordinal,
original Gazebo sequence, **signed** acquisition ns, assigned HRT µs, SensorGps
publication label µs, SensorGps sample label µs; then device uint32 [96:100],
receiver int32 [100:104], canonical payload [104:154], status [154], publication
boolean [155], velocity-valid boolean [156], zero padding [157:160], three host
monotonic diagnostic ns fields [160:184], zero reserved [184:192]. Status 1 is
initialization without publication, 2 is a publication observation, 3 is invalid
incoming metadata. Capture publication failure without inventing a successful
receiver advertisement. The SensorGps publication label is the existing entry
HRT; no actual publication-time HRT is synthesized.

Payload is the existing F.1 `<4d3fIBB` encoding (50 bytes): latitude, longitude,
MSL/ellipsoid altitude, N/E/D velocities, device ID, fix type and satellites.
It copies native object representations; signed zero survives. Do not use raw
C++ structure padding, protobuf byte order, rounded JSON or timestamps as keys.

The 128-byte footer contains `ATGPSF1\0` [0:8], context digest [8:40], eight
uint64 counters [40:104]: callbacks, enqueued, drained, overflow/contention,
invalid metadata, write errors, conservative queue high-water, shutdown failure;
CRC32 [104:108], diagnostic queue-operation max ns [108:116], observed callback
max ns [116:124], zero reserved [124:128]. CRC covers all event bytes followed
by the entire footer with its CRC field zeroed. Missing footer, partial writes,
nonzero loss/errors, differing counts or invalid checksums fail acceptance.

## Runtime interface, for separately approved execution only

- `AEROTWIN_GPS_OBSERVER`: absent/0 disables; exactly 1 enables; other values fail initialization.
- `AEROTWIN_GPS_CONTEXT`: absolute path to the frozen 512-byte regular file; symlinks rejected.
- `AEROTWIN_GPS_FIFO`: absolute path to an already-created FIFO with a reader; regular files and symlinks rejected.
- `AEROTWIN_GPS_TIMING`: absent/0 disables host timing probes; exactly 1 enables; malformed values rejected.

An owned external sink must create/open the FIFO before the bridge starts.
The sink writes an exclusive regular evidence file outside PX4 and validates
framing on close. The PX4 writer uses nonblocking, PIPE_BUF-sized writes; SIGPIPE
is blocked only in that writer thread. A failed/slow/disconnected sink invalidates
capture without blocking GPS processing or terminating PX4. The writer drains
for up to two host seconds after quiescence; a failed final write means missing
footer and invalid evidence. A filesystem-stalled sink can be terminated by its
owner without unsafe PX4 thread cancellation. Sink timeout is bounded to 180
wall seconds and size to 64 MiB; partial evidence is retained.

Callbacks do one MPSC reservation CAS attempt. Full queue or producer contention
is an explicitly counted loss; there is no retry loop, callback mutex, allocation,
hashing or file I/O. Capacity is 8,192 records, slots <256 bytes. Consumer alone
advances tail; release/acquire slot generations protect reuse. A callback cannot
wait for another producer. Conservative high-water or timing maxima saturate
on diagnostic update contention rather than understate a bound. Shutdown requires
subscription quiescence before joining the writer. The retained topology has
Gazebo in a separate process; in-process NavSat publishers are outside the
initial lifecycle contract. F.4 found that the separate-process case is also
not fenced by unsubscribe. MPSC stress tests do not qualify Transport teardown.

## Offline tests

Compile `tests/observer_test.cpp` together with the patched helper in the
isolated Linux checkout, using real generated SensorGps/protobuf types:

```sh
g++ -std=c++17 -O2 -g -fno-signed-zeros -fno-trapping-math -freciprocal-math \
  -fno-math-errno -Wall -Wextra -Werror -pthread -include visibility.h \
  -I src/modules/simulation/gz_bridge -I build/px4_sitl_default \
  -I platforms/common -I platforms/common/include -I platforms/posix/include \
  -I src -I src/include /evidence/observer_test.cpp \
  src/modules/simulation/gz_bridge/GpsAcquisitionObserver.cpp \
  $(pkg-config --cflags --libs gz-msgs10) -o /tmp/observer_test
PYTHONPATH=/aerotwin-parent python3 /evidence/run_observer_tests.py /tmp/observer_test
```

`/aerotwin-parent` must contain the existing `aerotwin_experiments` package.
The driver creates only temporary synthetic context/FIFO fixtures and runs the
standalone test executable. It launches no simulator or PX4 process. Python
regressions are `python -m pytest tests/python/test_m3_gps_hook.py`.

Restore by selecting the original immutable image and its original launch
manifest after stopping only owned F.3/F.4 containers. Verify the original PX4
hash; never overwrite its binary or reverse-patch its source. Keep review image,
patch, raw evidence and provenance independently identifiable.
