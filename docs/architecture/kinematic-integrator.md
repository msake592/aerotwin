# Constant-velocity kinematic integration

M1.7 merged in [PR #11](https://github.com/msake592/aerotwin/pull/11). Include
`<aerotwin/simulation/KinematicIntegrator.hpp>` and link `aerotwin_core`.
`aerotwin::integrateConstantVelocity(initial_state, target_timestamp)` is a pure
operation that returns `std::optional<VehicleState>`. It propagates simulator
ground truth. The caller supplies event times and owns clock advancement, resets,
and [scenario scheduling](scenario-telemetry.md).

## Motion and frames

For elapsed seconds `dt`, position in world ENU metres advances by
`p_next = p_current + v_world * dt`. World ENU linear velocity in metres per
second and body FLU angular velocity in radians per second are held constant
and copied to the output.

Let `h = omega_body * (dt / 2)` and `a = norm(h)`. The body increment is a
Hamilton quaternion with scalar part `cos(a)` and vector part
`h * (sin(a) / a)`, with the continuous limit at zero. The attitude update is
`q_next = q_current * delta_q_body`. This applies the body increment before the
current active body-to-world rotation, using
`composeRotations(q_current, delta_q_body)`. Zero angular velocity preserves the
original orientation coefficients, including accepted norm drift. A nonzero
representable increment uses M1.3's checked composition and bounded norm
correction. See [rotation conventions](quaternion-rotations.md).

## Time and failures

The initial snapshot already guarantees finite components, valid quaternion norm
and nonnegative elapsed time. The target must have the same epoch and an elapsed
count at least as large as the initial count. The caller supplies timestamps from
the same clock history. Subtraction in integer nanoseconds is safe across the
full nonnegative range and takes place before conversion to seconds. An equal
timestamp returns an exact copy of the initial state. Success at a later time
stores the exact supplied timestamp.

Invalid time progression, unrepresentable angular increments, failed checked
composition, and output that fails the vehicle state factory return
`std::nullopt`. The input remains unchanged on every path.

## Numerical limits

Position uses a fused multiply-add per coordinate, so a finite result can survive
cancellation even if the separate product would overflow. The rotation scales by
elapsed time before computing a three-argument hypotenuse. For half angles below
`1e-8`, the vector part uses the small-angle limit to avoid division. These are
numerical choices, with no physical speed or angular rate bounds.

Conversion of a long elapsed interval to double seconds rounds the interval.
An angular increment can underflow or become too large to represent, and phase
accuracy degrades for very large angles because the rounded angle is passed to
the standard trigonometric functions. Small increments may round away when
applied to an existing orientation. Repeated operations with identical inputs in
the same floating-point environment are deterministic. Step subdivision and
different math libraries can yield different rounded results.
