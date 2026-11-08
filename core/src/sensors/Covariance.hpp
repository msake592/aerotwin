#pragma once

#include <Eigen/Core>

#include <optional>

namespace aerotwin::detail {

// Returns the accepted symmetric covariance. If requested, also computes a
// sampling factor, projecting tolerated negative correlation eigenvalues to zero.
std::optional<Eigen::Matrix3d> validatedCovariance(const Eigen::Matrix3d& input,
                                                  Eigen::Matrix3d* sampling_factor = nullptr);

} // namespace aerotwin::detail
