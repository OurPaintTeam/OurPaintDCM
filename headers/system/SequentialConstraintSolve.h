#ifndef OURPAINTDCM_HEADERS_SYSTEM_SEQUENTIALCONSTRAINTSOLVE_H
#define OURPAINTDCM_HEADERS_SYSTEM_SEQUENTIALCONSTRAINTSOLVE_H

#include "ConstraintDecomposition.h"
#include "SolvePipeline.h"
#include <functional>

namespace OurPaintDCM::System {

// Restores all touched parameters on failure or exception. Success still needs the caller's
// model validation and coincidence synchronization. Only variables used by solved functions freeze.
bool solveSequentialConstraints(const RequirementSystem& system, const ConstraintDecomposition& plan,
    std::span<const std::size_t> order, const std::unordered_set<VAR>& locked,
    Utils::SolverType solver, double tolerance, const std::function<void(TaskMatrix&)>& optimize);

} // namespace OurPaintDCM::System

#endif
