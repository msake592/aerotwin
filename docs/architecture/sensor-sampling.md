# Deterministic sensor sampling

M2.5 adds a synchronous sampling pipeline in
`<aerotwin/sensors/SensorSamplingPipeline.hpp>`. Link `aerotwin_core`.
`SensorSamplingPipeline::create(gps, imu)` accepts optional typed configurations
with positive integer-nanosecond periods. Each configuration wraps an existing
validated sensor configuration. At least one sensor must be enabled. The
pipeline constructs and owns one instance of each enabled sensor and its RNG
streams. Omit a configuration to disable its channel. Registration is fixed
for the lifetime of the pipeline. GPS and IMU keep their existing generation APIs.

## Schedule and truth

The schedule starts in epoch 0. Each enabled channel is due at elapsed zero
and each exact multiple of its own period. After reset, elapsed zero is due
again. Submit a validated truth snapshot with the exact acquisition timestamp.
Supply matching `ImuKinematics` when IMU is due. On other calls the kinematics
argument is ignored. Callers supply inputs from the same intended run, and
matching timestamps alone cannot establish provenance.

Successful submissions must strictly increase in elapsed time within the
active epoch. A duplicate or backward timestamp returns an error. A non-due
timestamp succeeds with an empty batch and records the time. If a submission
skips any due timestamp, the entire call returns `MissedSample`. It can be
retried at the missed time with matching truth. The pipeline never uses later
truth for an earlier sample and performs no interpolation or extrapolation.

To recover, compute `next_due = t + (P - (t % P))` for each enabled period `P`,
where `t` is the last accepted elapsed time, including an accepted empty batch.
Use checked addition or compare the distances `P - (t % P)` before forming a
timestamp. Retry at the earliest missed boundary across enabled sensors. Before
any accepted submission, the first due boundary is zero.

For example, with a 10 ms IMU period, a jump from accepted 0 ms to 25 ms
returns `MissedSample` and leaves the schedule and RNG streams unchanged.
Obtain exact-time truth and IMU kinematics for 10 ms, retry there, and submit
the inputs for 20 ms before retrying at 25 ms. Recovery requires an exact-time
truth provider or replay from an earlier simulation state. The pipeline does
not reconstruct historical truth.

Scheduling compares elapsed distances. From the last accepted elapsed time
`t` the next due distance is `period - (t % period)`. The comparison uses
`submitted - t` after ordering and timestamp validation. These operations fit
in the existing signed nanosecond type. A due time beyond the representable
range cannot be submitted. The pipeline does not form that timestamp, so
there is no wrap or drift, and a due time at the maximum value is accepted.

## Output and ordering

`sample(truth, timestamp, kinematics)` returns a `SamplingResult` containing
either a `MeasurementBatch` or a `SamplingError`. The batch has two inline
optional slots. Occupied entries are contiguous from slot zero and hold
`GpsMeasurement` or `ImuMeasurement` variants. When both sensors are due,
GPS precedes IMU. The caller owns the returned values, including each
sensor ID, acquisition timestamp, and canonical covariance. Measurements
carry no reference to truth. Later estimator code can consume these
observations through the measurement contracts. Delivery time and latency
are outside this step.

## Failures, reset, and replay

Predictable errors are checked before sampling. Due sensors run on copies
of their current state. If either generator returns failure, no batch is
returned and the owned streams and last accepted timestamp remain unchanged.
In particular, a GPS result generated before a failed IMU sample is discarded
with its trial stream. Once every due sensor succeeds, the pipeline commits
the trial states and timestamp. The rollback guarantee applies to returned
sampling errors. Allocation or other C++ exceptions are outside that
guarantee. Copying due sensors has a per-sample cost and keeps transaction
logic independent of sensor internals.

`reset(epoch)` explicitly selects the epoch, restarts the schedule at zero,
and resets every owned stream to its configured seed. An epoch change never
resets implicitly. The pipeline does not enforce monotonic epoch progression.
The same or a lower epoch may be selected for replay. `SimulationClock` advances
its own epoch on reset. The caller coordinates the pipeline reset with the
clock. Matching timestamps alone establish neither run provenance nor physical
consistency of the supplied snapshots. Identical configuration, inputs,
executable, and floating-point environment reproduce the same ordered batches.

The deterministic integration test submits stationary truth and zero
reference-point acceleration at exact 10 ms intervals over the inclusive
interval `[0 s, 2 s]`. GPS at 1 Hz yields 3 samples, and IMU at 100 Hz yields
201. At 0, 1, and 2 s the GPS entry comes first. M1's motion demo and truth
export remain separate from sensor sampling.
