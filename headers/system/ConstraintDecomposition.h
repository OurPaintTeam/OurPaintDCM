#ifndef OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTDECOMPOSITION_H
#define OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTDECOMPOSITION_H

#include "ArticulationDecomposition.h"
#include <cstddef>
#include <optional>
#include <vector>

namespace OurPaintDCM::System {

class RequirementSystem;

struct ConstraintBlock {
    std::vector<Utils::ID> figures;
    std::vector<Utils::ID> requirements;
    std::vector<std::size_t> functionIndices;
};

struct ConstraintDecomposition {
    std::vector<ConstraintBlock> blocks;
    std::vector<Utils::ID> articulationPoints;
    std::vector<double*> sharedVariables;
};

/** Bind a local graph decomposition to its original requirement system.
 * Each requirement and function has one owner. Empty blocks retain topological connections.
 * nullopt requests ordinary solving when separation is unsafe or fewer than two blocks have functions.
 * The plan stores no numerical values or function copies. Rebuild it when the system's function list,
 * point aliases or geometry structure change; numerical success still requires residual validation.
 */
[[nodiscard]] std::optional<ConstraintDecomposition> buildConstraintDecomposition(
    const RequirementSystem& system, ArticulationDecomposition decomposition);

} // namespace OurPaintDCM::System

#endif
