# Deterministic Gaussian noise

M2.2 provides `GaussianNoise` and `GaussianNoise3` in
`<aerotwin/sensors/GaussianNoise.hpp>`. The caller owns the seed, stream,
and sample order. The noise layer has no clock, truth-state, or scheduling
dependency. Future GPS and IMU models can own separate streams and add samples
in the measurement's frame and units.

## Scalar stream

`GaussianNoise(std::uint64_t seed)` owns a `std::mt19937_64` engine seeded
directly from the caller's value. The engine's integer algorithm and seeding
are standardized. It is suitable for simulation replay and is non-cryptographic.

Each standard normal draw consumes two engine words. The high 52 bits of each
word map to the exact midpoint `(bits + 0.5) * 2^-52` in the open unit interval.
Box–Muller returns `sqrt(-2 log(u)) * cos(2 pi v)`. The sine partner is discarded
so every draw has the same consumption. The explicit transform avoids
implementation-dependent distribution algorithms.

`sample(sigma)` multiplies a standard normal by a finite positive standard
deviation. Zero returns exactly zero without advancing the stream. Negative
or non-finite parameters return an empty optional without consuming a draw.
If multiplication overflows, the draw remains consumed and the result is
empty. Copies snapshot the current engine state. Construction or assignment
from an rvalue also copies the engine state and leaves the source usable and
unchanged. `reseed(seed)` restarts the stream from the supplied seed.

## Covariance sampling

`GaussianNoise3::create(covariance)` factors a validated 3×3 covariance once.
It shares the [M2.1 covariance policy](measurements.md#validation), including
pair-local symmetry tolerance, exact zero-variance isolation and
variance-normalized PSD validation. An invalid covariance returns an empty
optional without involving a stream.

With standard deviations in diagonal matrix `D`, let the normalized
correlation matrix have eigendecomposition `Q diag(lambda) Q^T`. The cached
factor is `D Q diag(sqrt(max(lambda, 0)))`. Each call draws three standard
normals in order and multiplies the factor by that vector. This supports
singular PSD and zero covariance. Zero covariance still consumes three draws.
A non-finite result returns an empty optional after the draws.

Factoring the correlation matrix preserves scale across mixed variances and
returns samples in the original units. Eigenvalues already accepted in
`[-1e-12, 0)` are projected to zero for sampling because negative variances
cannot be realized. In exact arithmetic the spectral norm of this adjustment
in correlation space is at most `1e-12`. The measurement contract still stores
its accepted symmetric covariance without this projection. Floating-point
rounding can affect near-null directions, underflow tiny normalized entries,
and change the realized covariance slightly. The finite uniform grid also
bounds the sampled tails.

## Reproducibility

Identical seeds, implementation, executable, floating-point environment,
parameters, and call order reproduce identical outputs. A seed alone does not
identify a sample. Math library rounding and Eigen eigenvector signs or bases
can differ between builds and platforms. The standardized integer sequence
does not imply bitwise cross-platform Gaussian or correlated output.
