# M3.1 environment validation

Activate the [editable Python environment](../../README.md#python-development-environment).

```sh
python -m aerotwin_experiments.m3_preflight --profile macos-dev --json
python -m aerotwin_experiments.m3_preflight --profile linux-integration
```

Inventory checks platform, tools and version output. The Linux profile
expects Ubuntu 24.04, Gazebo Sim 8 (Harmonic), a sourced Jazzy CLI, and XRCE
Agent discovery. ARM64 receives a support warning. An Agent path does not
verify its version. Inventory launches only diagnostic commands.

Exit 0 means executed checks have no failures or blockers, exit 1 means a
command failed or timed out, and exit 2 means missing prerequisites or
invalid arguments. Reports use PASS, FAIL, BLOCKED, SKIP, WARN and INFO.
The JSON form includes the command arguments when applicable.

## Prepare an existing Ubuntu environment

Select the [candidate stack](../reports/m3-integration-readiness.md#candidate-stack-compatibility):
Ubuntu 24.04 amd64, ROS Jazzy, Harmonic, PX4 v1.17.0 at the audited SHA
and standalone Agent v2.4.3. Review package origins and availability
before provisioning. Record exact package revisions and source/submodule
SHAs or an image digest. A version substitution requires fresh validation.
Keep processes on one host and use an isolated ROS domain.

- Select `ros-jazzy-ros-gz` with Harmonic and check the bridge and
  `gz-sim8` versions resolved on the target host. [ROS/Gazebo installation](https://gazebosim.org/docs/harmonic/ros_installation/)
- Prepare a trusted `px4_sitl_default` build from the tagged checkout.
  Review the tagged setup instructions before using them and select
  Harmonic explicitly. [PX4 Ubuntu guide](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu)
- Source Jazzy and an overlay with firmware-matched `px4_msgs` definitions.
  Check the message files and DDS topic list against the selected firmware.
  Record the Agent and its resolved dependency revisions.
  [PX4 DDS guide](https://docs.px4.io/v1.17/en/middleware/uxrce_dds)

## Opt-in checks

```sh
python -m aerotwin_experiments.m3_preflight --profile linux-integration --smoke --json
python -m aerotwin_experiments.m3_preflight --profile linux-integration --smoke \
  --px4-checkout "$PX4_CHECKOUT" --json
```

Gazebo uses an asset-free temporary SDF, private partition and ten requested
server-only iterations with a 30-second timeout. The probe requires world
initialization and shutdown diagnostics, since exit zero alone can miss
startup failures. It does not independently observe the step count. PX4 checks the supplied
checkout and submodule status, then invokes the existing binary with
`-h`. It checks executable availability and dynamic loading. Build
provenance must come from the manifest. No command installs dependencies
or starts PX4 SITL or its Agent. ROS checks the bridge and message interfaces.

Once an already validated external session is publishing, request a
bounded read-only DDS sample from its actual SensorCombined topic:

```sh
python -m aerotwin_experiments.m3_preflight --profile linux-integration --smoke \
  --px4-checkout "$PX4_CHECKOUT" --discover-topic /fmu/out/sensor_combined --json
```

The sample probe uses sensor-data QoS and no daemon and times out after
15 seconds. Diagnose missing samples by checking domain, discovery scope,
RMW, message definition, offered QoS and rate. A received sample alone
does not validate clocks or simulation provenance.

Each command runs in a private POSIX process group with bounded waits
and group cleanup. Missing checks are explicit. Unit tests require
no robotics software. Successful mocks do not establish runtime readiness.

For the pinned setup and two-run x500/odometry acceptance procedure, see the
[M3.2 runtime guide](m3-sitl-runtime.md).
