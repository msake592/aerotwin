#pragma once

#include "FusionEvaluation.hpp"

#include <iosfwd>
#include <string>
#include <string_view>

namespace aerotwin::fusion_demo {

// The argument is a complete, successful result returned by evaluateFusion().
[[nodiscard]] std::string serializeFusionCsv(const FusionEvaluation& evaluation);

// Runs evaluation and serializes in memory before writing any report bytes.
int writeFusionReport(AcquisitionResult acquisition, std::ostream& output, std::ostream& errors);

namespace detail {
void writeCsvField(std::ostream& stream, std::string_view field);
}

} // namespace aerotwin::fusion_demo
