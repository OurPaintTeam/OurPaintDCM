#ifndef OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTRANKDIAGNOSIS_H
#define OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTRANKDIAGNOSIS_H

#include "ConstraintLinearization.h"

namespace OurPaintDCM::System {

struct ConstraintRankDiagnosis {
    enum class Status { DEFINED, NUMERICALLY_UNDEFINED };

    Status status = Status::NUMERICALLY_UNDEFINED;
    std::optional<std::size_t> rank;
    // Row indices into the input snapshot, excluding zero-weight equations.
    std::vector<std::size_t> independentEquations;
    std::vector<std::size_t> dependentEquations;
};

/** Choose a local differential basis using one rank-revealing sparse QR of J^T.
 * Equilibrates equation and coordinate scales before applying a relative tolerance.
 * Dependence says nothing about compatibility or global nonlinear redundancy.
 * No factorization or nonlinear solve is performed by ordinary solve/DRAG paths.
 */
[[nodiscard]] ConstraintRankDiagnosis diagnoseConstraintRank(
    const ConstraintLinearization& linearization, double relativeTolerance = 1e-8);

} // namespace OurPaintDCM::System

#endif
