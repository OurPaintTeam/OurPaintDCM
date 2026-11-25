#ifndef OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTSOLVEORDER_H
#define OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTSOLVEORDER_H

#include "ConstraintDecomposition.h"
#include <span>

namespace OurPaintDCM::System {

/** Choose a numerical root, then traverse incident blocks through shared vertices.
 * Priorities: fixed parameters, edited geometry, local degrees of freedom, structure.
 * Uses current geometry; the returned order includes connector blocks exactly once.
 */
[[nodiscard]] std::vector<std::size_t> chooseConstraintSolveOrder(
    const RequirementSystem& system, const ConstraintDecomposition& plan,
    std::span<const Utils::ID> editedFigures = {});

} // namespace OurPaintDCM::System

#endif
