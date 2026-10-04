#ifndef OURPAINTDCM_HEADERS_UTILS_NUMERICVALIDATION_H
#define OURPAINTDCM_HEADERS_UTILS_NUMERICVALIDATION_H

#include "Enums.h"
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>

namespace OurPaintDCM::Utils {

constexpr void requireFinite(double value) {
    // Comparisons also reject NaN and remain usable in C++20 constexpr constructors.
    constexpr double limit = std::numeric_limits<double>::max();
    if (!(value >= -limit && value <= limit)) {
        throw std::invalid_argument("Numeric value must be finite");
    }
}

constexpr void requireFinite(const std::optional<double>& value) {
    if (value) requireFinite(*value);
}

template <typename Range>
constexpr void requireFiniteValues(const Range& values) {
    for (const auto& value : values) requireFinite(value);
}

constexpr void requireNonNegative(double value) {
    requireFinite(value);
    if (value < 0.0) {
        throw std::invalid_argument("Numeric value must be non-negative");
    }
}

constexpr void requirePositiveRadius(double radius) {
    requireFinite(radius);
    if (radius <= 0.0) {
        throw std::invalid_argument("Circle radius must be positive");
    }
}

inline void validateRequirementParameter(RequirementType type, const std::optional<double>& param) {
    switch (type) {
        case RequirementType::ET_POINTLINEDIST:
        case RequirementType::ET_POINTPOINTDIST:
        case RequirementType::ET_LINECIRCLEDIST:
            if (!param) throw std::invalid_argument("Distance requirement needs a parameter");
            requireNonNegative(*param);
            break;
        case RequirementType::ET_LINELINEANGLE:
            if (!param) throw std::invalid_argument("Angle requirement needs a parameter");
            requireFinite(*param);
            if (*param < 0.0 || *param > std::numbers::pi) {
                throw std::invalid_argument("Line angle must be in [0, pi] radians");
            }
            break;
        default:
            if (param) throw std::invalid_argument("Requirement does not accept a parameter");
            break;
    }
}

} // namespace OurPaintDCM::Utils
#endif
