# Deterministic scenarios and truth telemetry

M1.8 is implemented locally and pending technical review.

`ScenarioConfig::create(initial_state, timestep, step_count, seed, identifier)`
validates a caller-owned configuration. The initial `VehicleState` must match a
fresh `SimulationClock` at epoch 0 and elapsed 0 ns. The positive integer
nanosecond timestep is required even for zero steps. Step count is signed at the
API boundary. Accepted counts are 0 through 100,000, and the product of step
count and timestep must fit in signed 64-bit nanoseconds. The limit bounds the
runner's single in-memory trajectory to 100,001 snapshots, roughly 12 MB of
sample storage on the supported 64-bit builds. An optional identifier is 1–64
ASCII letters, digits, periods, hyphens, or underscores. The unsigned 64-bit
seed is reserved for future stochastic components and has no effect on M1 motion.

## Execution and ownership

`runScenario(config)` creates and owns a fresh clock, records the exact initial
snapshot at sample zero, then explicitly steps the clock and delegates every
propagation to `integrateConstantVelocity`. The integrator returns validated
`VehicleState` values, which are appended in order. Success produces one
move-only `ScenarioResult` with the copied configuration and exactly `step_count + 1`
independently owned samples, exposed through read-only accessors. Zero steps
produce only the initial snapshot. Clock, integration, and allocation failures
return an empty optional and discard any accumulated samples. The runner neither
mutates the configuration nor performs file I/O.

## Schema version 1

`exportTruthTelemetry(result, output_directory)` uses POSIX directory descriptors
on macOS and Ubuntu. The existing parent must be owned by the effective user
or root. Group- or world-writable parents require the sticky bit. The exporter
creates a private leaf relative to the opened parent and rejects existing leaves,
including symlinks. It checks the new leaf's owner and mode, opens it without
following a symlink, and verifies its device and inode. The retained descriptor
anchors exclusive, no-follow file creation and cleanup. Each output file is
created with owner-only permissions. Writes handle interruptions and short
writes, and success requires both file descriptors to close successfully.

Before returning success, the exporter verifies the parent and destination
identities and both output entries. If the destination was renamed or replaced,
it reports failure. Cleanup uses the original directory descriptor and removes
only entries whose identities match files created by this export. The leaf
directory can remain empty or contain residual owned artifacts after failure.
POSIX provides no atomic identity-checked directory removal, so the exporter
leaves the leaf itself in place.

The ownership checks assume normal POSIX permission and sticky-bit enforcement
and a trusted root, effective user, and directory ACL configuration. A process
that can modify the private directory, including one running as the same user,
can race an identity check and subsequent unlink. POSIX also has a gap between
directory creation and the first identity observation. The parent restrictions
protect that gap from other ordinary users under the stated permission assumptions.
The final identity check observes the path at a point in time. It cannot prevent
a later rename. Use a private parent when other processes can run as the same user.

The two-file export is non-transactional. Files can be visible while writing,
and process interruption or cleanup failure can leave incomplete output. The
export does not synchronize files to durable storage. Consumers should use the
directory after a successful return.

- `truth_trajectory.csv` begins with a fixed header and one row per snapshot in
  sample order. `epoch` and `elapsed_ns` are exact integer fields.
  `position_{e,n,u}_m` and `velocity_{e,n,u}_mps` are world ENU vectors.
  `orientation_{w,x,y,z}` is the dimensionless active Hamilton quaternion from
  body FLU to world ENU, in WXYZ order. `angular_velocity_{f,l,u}_radps` is body
  FLU angular velocity. Rows use LF and doubles use the classic locale with
  `max_digits10` precision.
- `run_metadata.txt` uses LF-delimited `key=value` records. Keys are
  `schema_version`, `scenario_id` (empty if omitted), `seed`, `timestep_ns`,
  `step_count`, `sample_count`, `world_frame`, `body_frame`, `orientation`,
  `time_unit`, `position_unit`, `linear_velocity_unit`, and
  `angular_velocity_unit`.

The schema carries simulator truth only. Future measurement and estimate
contracts remain separate. There is no wall-clock scheduling or timestamping.
The same configuration, executable, and floating-point environment reproduce
the same samples and CSV bytes. Different seeds preserve the same M1 truth
sequence. The [integrator limits](kinematic-integrator.md#numerical-limits)
apply. Floating-point bitwise agreement across toolchains or architectures is
outside the reproducibility contract.
