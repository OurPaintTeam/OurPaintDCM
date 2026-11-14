#ifndef OURPAINTDCM_HEADERS_SYSTEM_SOLVEPIPELINE_H
#define OURPAINTDCM_HEADERS_SYSTEM_SOLVEPIPELINE_H

#include "RequirementFunction.h"
#include "TaskMatrix.h"
#include <span>
#include <unordered_map>
#include <unordered_set>

namespace OurPaintDCM::System {

using FixedAssignments = std::unordered_map<VAR, double>;
using RequirementBindings = std::span<const std::shared_ptr<Function::RequirementFunction>>;

struct SolvePipeline {
    std::vector<std::unique_ptr<Variable>> variableOwners;
    std::unique_ptr<TaskMatrix> task;
    bool hasFunctions = false;
    bool hasFreeVariables = false;
};

FixedAssignments collectFixedAssignments(RequirementBindings functions);

// Fixed and locked coordinates remain live inputs, excluded from the optimization variables.
// Preparation does not assign coordinates; callers apply fixed targets before constructing tasks.
SolvePipeline buildSolvePipeline(RequirementBindings functions, const FixedAssignments& fixed,
    const std::unordered_set<VAR>& locked, Utils::SolverType solver);

} // namespace OurPaintDCM::System

#endif
