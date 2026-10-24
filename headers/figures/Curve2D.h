#ifndef OURPAINTDCM_HEADERS_FIGURES_CURVE2D_H
#define OURPAINTDCM_HEADERS_FIGURES_CURVE2D_H

#include "ID.h"
#include "NumericValidation.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace OurPaintDCM::Figures {

/** NURBS with an expanded knot vector (including repeated knots).
 * Non-periodic: N control points, N + degree + 1 knots.
 * Periodic: N controls without a duplicated cyclic tail, N + 2*degree + 1 knots;
 * controls repeat cyclically and knots[i + N] = knots[i] + period.
 */
struct Curve2D {
    unsigned degree = 0;
    std::vector<Utils::ID> pointIds;
    std::vector<double> weights;
    std::vector<double> knots;
    bool periodic = false;

    static void validateData(unsigned degree, std::size_t count,
                             const std::vector<double>& weights,
                             const std::vector<double>& knots, bool periodic) {
        if (degree == 0 || count <= degree || weights.size() != count ||
            knots.size() != count + std::size_t(degree) * (periodic ? 2 : 1) + 1) {
            throw std::invalid_argument("Invalid curve degree, control count, weights or knot count");
        }
        Utils::requireFiniteValues(weights);
        Utils::requireFiniteValues(knots);
        for (double weight : weights) {
            if (weight <= 0) throw std::invalid_argument("Curve weights must be positive");
        }
        if (!std::is_sorted(knots.begin(), knots.end())) {
            throw std::invalid_argument("Curve knots must be non-decreasing");
        }
        Utils::requireFinite(knots.back() - knots.front());
        const double period = knots[count + (periodic ? degree : 0)] - knots[degree];
        if (period <= 0) throw std::invalid_argument("Curve parameter range must be non-empty");
        std::size_t multiplicity = 0;
        for (std::size_t i = 0; i < knots.size(); ++i) {
            multiplicity = i && knots[i] == knots[i - 1] ? multiplicity + 1 : 1;
            const bool end = !periodic && (knots[i] == knots.front() || knots[i] == knots.back());
            if (multiplicity > std::size_t(degree) + end) {
                throw std::invalid_argument("Curve knot multiplicity exceeds its degree");
            }
        }
        if (periodic) {
            for (std::size_t i = 0; i + count < knots.size(); ++i) {
                const double tolerance = 64 * std::numeric_limits<double>::epsilon() *
                    std::max({1.0, std::abs(knots[i]), std::abs(knots[i + count]), period});
                if (std::abs((knots[i + count] - knots[i]) - period) > tolerance) {
                    throw std::invalid_argument("Periodic curve knots must repeat by the period");
                }
            }
        }
    }

    void validate() const { validateData(degree, pointIds.size(), weights, knots, periodic); }
    double firstParameter() const { return knots.at(degree); }
    double lastParameter() const { return knots.at(pointIds.size() + (periodic ? degree : 0)); }
};

} // namespace OurPaintDCM::Figures
#endif
