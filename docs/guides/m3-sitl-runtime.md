# M3.2 PX4 and Gazebo runtime validation

**Status: PARTIAL.** The runner and portable tests are available. Live evidence
requires an isolated Ubuntu 24.04 amd64 host and is pending. The
[M3.1 preflight](m3-environment.md) remains the first inventory step.

## Reference candidates

Upstream release and source checks were performed on 2026-11-14. PX4 v1.17.0
was the latest stable release. Component selections below are candidates until
the live gates pass.

| Component | Candidate and required evidence |
| --- | --- |
| Ubuntu | 24.04 LTS amd64. Record image digest or host release, architecture, package origins and installed revisions. ARM64 and native macOS are outside this runner's reference profile. |
| ROS 2 | Jazzy, including ros-base, ros-gz bridge, rclpy and Fast DDS RMW. Record exact Debian versions from the ROS repository. [ROS/Gazebo pairing](https://gazebosim.org/docs/harmonic/ros_installation/) |
| Gazebo | Harmonic, gz-sim 8, gz-transport 13, SDFormat 14. Record exact library versions and provenance. Use the Jazzy Harmonic vendor stack where it supplies the CLI and build libraries. Review compatible OSRF Harmonic packages if needed and record the resolution. Reject competing gz-sim installations in the selected environment. [Harmonic library matrix](https://gazebosim.org/docs/harmonic/install/) |
| PX4 | v1.17.0, `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, with its recorded recursive submodules. [PX4 release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0) |
| Micro XRCE-DDS Agent | v2.4.3, `73622810d984349b80bbac0ef55fc0b694d62222`. Build standalone with private Fast DDS and Fast CDR dependencies. Record resolved dependency SHAs. Agent 3.x is incompatible with the chosen 2.x PX4 client. [PX4 XRCE guide](https://docs.px4.io/v1.17/en/middleware/uxrce_dds) |
| px4_msgs | `86d8239e962f6939e05c3737784f60c02fa884db`. SensorGps, SensorCombined and VehicleOdometry match the firmware's source blobs. The runner compares those definitions with the installed overlay. [Pinned definitions](https://github.com/PX4/px4_msgs/tree/86d8239e962f6939e05c3737784f60c02fa884db) |

## Prepare an existing host

Plan for at least 16 GiB RAM and 50 GiB free disk as development capacity
estimates. This package has no measured minimum. Use a dedicated host or guest
with no physical vehicles, serial devices, ground-control software or other
SITL sessions. Isolate it from vehicle and control networks. The runner uses
loopback Gazebo discovery, a unique Gazebo partition and PX4 namespace, an
explicit XRCE UDP port, and a selected local ROS domain. PX4's default startup
also opens MAVLink ports. The XRCE Agent UDP listener is not a firewall.
Use a dedicated test network. No arming or control command is sent.

Set up the approved Ubuntu, ROS Jazzy and, when required, OSRF Harmonic
repositories separately. Inspect package candidates before installation.

```sh
dpkg --print-architecture
apt-cache policy ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-rmw-fastrtps-cpp \
  gz-harmonic libgz-sim8-dev libgz-transport13-dev libsdformat14-dev
gz sim --versions
```

Select exact versions from approved origins. Use `apt-get -s install
package=version` for the proposed resolution, review transitive dependencies,
and record the installed versions after the operator provisions the host.
An unavailable version needs explicit review. The runner never invokes
installation. Verify that PX4's CMake configuration resolves Harmonic
development libraries and produces Gazebo plugins before proceeding.

Use a **new directory outside AeroTwin** and checkout by SHA. The following
commands explicitly download and build external sources. Execute only after
reviewing host prerequisites and dependency sources.

```sh
# Set STACK to a new directory outside the AeroTwin checkout.
mkdir -p "$STACK/ros-overlay/src"
git clone --no-checkout https://github.com/PX4/PX4-Autopilot.git "$STACK/PX4-Autopilot"
git -C "$STACK/PX4-Autopilot" checkout --detach d6f12ad1c4f70ad3230afd7d86e971421e02fef4
test "$(git -C "$STACK/PX4-Autopilot" rev-parse v1.17.0^{commit})" = \
  d6f12ad1c4f70ad3230afd7d86e971421e02fef4
git -C "$STACK/PX4-Autopilot" submodule update --init --recursive

git clone --no-checkout https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$STACK/agent"
git -C "$STACK/agent" checkout --detach 73622810d984349b80bbac0ef55fc0b694d62222
test "$(git -C "$STACK/agent" rev-parse v2.4.3^{commit})" = \
  73622810d984349b80bbac0ef55fc0b694d62222
cmake -S "$STACK/agent" -B "$STACK/agent-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$STACK/agent-install"
cmake --build "$STACK/agent-build" --parallel 4
# The superbuild stages the Agent and dependencies in its configured prefix.
# Inspect the installed binary and dynamic libraries before starting a run.

git clone --no-checkout https://github.com/PX4/px4_msgs.git "$STACK/ros-overlay/src/px4_msgs"
git -C "$STACK/ros-overlay/src/px4_msgs" checkout --detach 86d8239e962f6939e05c3737784f60c02fa884db
source /opt/ros/jazzy/setup.bash
(cd "$STACK/ros-overlay" && colcon build --packages-select px4_msgs)
source "$STACK/ros-overlay/install/setup.bash"

make -C "$STACK/PX4-Autopilot" -j4 px4_sitl_default
cmake --build "$STACK/PX4-Autopilot/build/px4_sitl_default" --target px4_gz_plugins
```

Review and capture the Agent superbuild dependency revisions, package
origins and artifact hashes. The v2.4.3 executable has no verified
standalone version-report flag. The runner verifies the trusted checkout
and binary hash against the operator's v2.4.3 build record. A matching
hash confirms artifact identity, while the build record establishes provenance.
The split build/run path reuses the x500 startup environment of
`HEADLESS=1 make px4_sitl gz_x500` and starts the already-built binary in a
fresh writable directory with an explicit rootfs. Runtime never builds
or downloads dependencies. PX4's x500 model and default world come from
the pinned simulation submodule. [Tagged Gazebo integration](https://docs.px4.io/v1.17/en/sim_gazebo_gz/)

## Record and run

Use Ubuntu's Jazzy-compatible Python ABI (Noble system Python 3.12). Create a
project development venv with access to the sourced ROS Python packages, using
`--system-site-packages` if needed. Install AeroTwin editable in that venv.
Source Jazzy and the verified overlay before invoking the CLI. Ensure the
Agent binary can find its private libraries without replacing ROS libraries.
Keep manifest, inventory and reports outside the source checkout.

```sh
python -m aerotwin_experiments.m3_preflight --profile linux-integration --smoke \
  --px4-checkout "$STACK/PX4-Autopilot" --json
python -m aerotwin_experiments.m3_runtime plan
python -m aerotwin_experiments.m3_runtime inventory > "$STACK/manifest.json"
```

Inventory writes `revisions`, `packages` and candidate `agent_version`.
Review them, then add these fields to the JSON manifest using values from the
actual build:

| Key | Value |
| --- | --- |
| `package_origins` | Nonempty repository and image/host provenance record. Save the apt policy output. |
| `agent_dependencies` | Nonempty record of resolved Agent dependency versions and SHAs. |
| `px4_binary_sha256` | SHA-256 of `build/px4_sitl_default/bin/px4`. |
| `agent_binary_sha256` | SHA-256 of the selected `MicroXRCEAgent` executable. |

```sh
python -m aerotwin_experiments.m3_runtime live --isolated-host \
  --manifest "$STACK/manifest.json" --px4 "$STACK/PX4-Autopilot" \
  --agent "$STACK/agent" --agent-binary "$STACK/agent-install/bin/MicroXRCEAgent" \
  --px4-msgs "$STACK/ros-overlay/src/px4_msgs" --domain 55 --port 18888 \
  > "$STACK/m3-runtime-report.json"
```

Use a dedicated unused domain and UDP port. The runner checks versions,
checkout revisions, package inventory, interface definitions and port
availability before starting external processes. Gazebo must advance, pause at
unchanged time, jump backwards on a time-only reset, then resume. This happens
only in an asset-free standalone world. The two x500 runs each require
progressing Gazebo statistics and three decoded, strictly ordered
VehicleOdometry observations. The JSON includes discovery, subscription
creation, receipt, decoding, semantic checks, offered/requested QoS, essential
fields, elapsed startup time and bounded stdout/stderr tails. The subscription
requests best-effort, volatile, keep-last depth five. Only one stable discovered
publisher is accepted in the run's private namespace. Successful decoded
traffic from the owned session supplies transport evidence. There is no
hardware connection.

Exit 1 means a probe failed. Exit 2 means a prerequisite or required
acceptance gate remains unresolved. Live gates report PASS, FAIL, BLOCKED
or NOT_RUN individually. The runner intentionally marks cross-domain mapping
NOT_RUN, even if the standalone clock and both x500 cycles pass. Process
groups are scoped to the owned subprocesses and logs are captured before
temporary working directories are removed. A forced cleanup fails the run.

## Clock and frame follow-up

Use the [adapter boundary](../architecture/m3-adapter-boundary.md#clock-and-transport-gates)
and an isolated session to compare Gazebo world clock, bridged ROS clock, and
PX4 publication and sample timestamps. Bridge Gazebo clock unidirectionally
with the Jazzy Harmonic `ros_gz_bridge` parameter bridge, using the actual
world clock topic and a ROS remap to `/clock` if required. Record the chosen
PX4 `UXRCE_DDS_SYNCT` configuration. Compare progression and offsets with
sim time enabled, then characterize pause, resume and reset without arming.
If reset requires restarting the coupled stack, record that requirement and
capture pre/post-reset clock and topic evidence. Runtime code does not apply
AeroTwin epochs or assume a shared time origin.

VehicleOdometry is PX4's internal estimate. The tagged interface documents
microsecond times, NED or heading-relative FRD pose, independent velocity
frame enums (including body FRD), Hamilton WXYZ attitude, and three variance
vectors. NaNs and unknown frame enums can mark unavailable data. The probe
checks usable pose/velocity frames, finite vectors, quaternion magnitude,
timestamp order and stable reset counter. Variance fields are not full
covariance, and this package performs no frame conversion. Test attitude
direction and clock mapping before implementing adapters.
[Tagged VehicleOdometry.msg](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/msg/versioned/VehicleOdometry.msg)

## Observed verification

The starting main commit was `e8ce27c210186823470a2ed6baf1dce68f24ebb4`
([merged PR #23](https://github.com/msake592/aerotwin/pull/23)). Local tests on
macOS 26.6.2 arm64 passed 230/230 in both CTest presets and 37/37 in pytest
(20 M3.2 cases). Ruff lint and formatting, pip check, local documentation
paths and Git whitespace checks passed. CTest used the standard Debug and
Release configure/build/test presets. The existing preflight passed the
macOS profile. The M3.2 plan exited 0; inventory and live exited 2 on this
host as expected. Docker's existing daemon was unreachable.

Gazebo startup and clock progression, both x500 repetitions, XRCE exchange,
ROS message decoding, and live cleanup are **BLOCKED** by the absent Ubuntu
runtime. Standalone-world pause/reset and cross-domain clock observations
were **NOT RUN**. Process cleanup was exercised with local test subprocesses.
No Ubuntu AMD64 reference package versions have been resolved or accepted yet.

## M3.2.2 Docker ARM64 results

Docker Desktop on Apple Silicon ran two unarmed, headless x500 sessions with
PX4 Release 1.17.0 at `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
Both used Ubuntu 24.04.4 ARM64, Gazebo 8.15.0, ROS Jazzy
(`ros-jazzy-ros-base` 0.11.0), the Agent v2.4.3 source pin, and the
`px4_msgs` pin listed above. The three checked message files matched the
firmware byte for byte. This run used the existing container based on
`px4io/px4-sitl-gazebo@sha256:29a5a1b5af77e5d39b67bc6664d36acead63f9dfb829475216522b1feec8036e`
for its Harmonic libraries. The packaged PX4 executable was bypassed.

To reproduce the source build inside an authorized ARM64 Docker workspace,
limit the container to two CPUs and 6 GiB of memory. Install the following
build packages inside the container, then use the pinned source and build
commands in this guide with `-j2` and `--parallel 2`. Install PX4's
`Tools/setup/requirements.txt` in a Python 3.12 virtual environment before
configuring PX4.

```sh
apt-get update
apt-get install --no-install-recommends git build-essential ninja-build cmake \
  python3-venv python3-dev python3-pip libssl-dev libxml2-dev libxml2-utils \
  pkg-config ca-certificates curl zip unzip libopencv-dev cppzmq-dev
```

Set `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` from the pinned
source and build before starting Gazebo. The base image's inherited resource
paths can otherwise take precedence over the pinned assets. Start `gz sim
-r -s` with the pinned default world in a separately managed process, set
`PX4_GZ_STANDALONE=1`, `PX4_GZ_WORLD=default`, and
`PX4_GZ_MODELS` to the pinned models path, then run the newly built PX4
binary with `HEADLESS=1` and `PX4_SIM_MODEL=gz_x500`.

The validated topology ran the Agent and Jazzy subscriber in the same
container as PX4, with DDS discovery restricted to localhost, UDP port
18888, and ROS domain 55. Before starting PX4, set
`PX4_UXRCE_DDS_PORT=18888` and `ROS_DOMAIN_ID=55` and export a fresh
`PX4_UXRCE_DDS_NS`. Start the built Agent with
`MicroXRCEAgent udp4 -p 18888 -v 4` using its private Fast DDS libraries.
From the same container, source Jazzy and the overlay. With
`ROS_DOMAIN_ID=55`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and the
experiment package importable by Jazzy's Python, run the probe.

```sh
python3 -m aerotwin_experiments.m3_odometry_probe \
  "/$PX4_UXRCE_DDS_NS/fmu/out/vehicle_odometry"
```

Both sessions connected and decoded three valid ordered
estimates from one publisher. PX4 and Gazebo exited with code 0 and the
Agent stopped with SIGTERM. The synced DDS timestamps were epoch-scale,
while PX4 sensor logs remained simulation-relative. The retained ULogs
showed no duplicate or backward sample times in the recorded gyro,
accelerometer, or vehicle IMU streams.

Standalone Gazebo time-only reset moved time backward and resumed the
world in repeated ARM64 probes. That behavior fails the existing clock
gate, which requires the reset to preserve pause. Cross-domain clock mapping,
coupled reset, separate-container transport, and the Ubuntu AMD64 reference
remain unverified.
