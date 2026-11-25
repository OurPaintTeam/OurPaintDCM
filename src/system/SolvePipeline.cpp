#include "SolvePipeline.h"
#include "SparseLSMTask.h"
#include "SparseSystemTask.h"

namespace OurPaintDCM::System {
namespace {
bool isSolverFunction(const Function::RequirementFunction& binding) {
    const auto type = binding.getType();
    return binding.getWeight() != 0 && type != Utils::RequirementType::ET_FIXPOINT &&
        type != Utils::RequirementType::ET_FIXARC && type != Utils::RequirementType::ET_FIXLINE &&
        type != Utils::RequirementType::ET_FIXCIRCLE;
}
}

FixedAssignments collectFixedAssignments(RequirementBindings functions) {
    FixedAssignments assignments;
    for (const auto& binding : functions) {
        VAR variable = nullptr;
        double target = 0;
        if (binding->tryGetAssignment(variable, target)) assignments[variable] = target;
    }
    return assignments;
}

std::vector<VAR> collectSolveVariables(RequirementBindings functions, const FixedAssignments& fixed,
    const std::unordered_set<VAR>& locked) {
    std::vector<VAR> variables;
    std::unordered_set<VAR> seen;
    for (const auto& binding : functions) {
        if (!isSolverFunction(*binding)) continue;
        for (auto* variable : binding->getVars()) {
            if (!locked.contains(variable) && !fixed.contains(variable) && seen.insert(variable).second)
                variables.push_back(variable);
        }
    }
    return variables;
}

SolvePipeline buildSolvePipeline(RequirementBindings functions, const FixedAssignments& fixed,
    const std::unordered_set<VAR>& locked, Utils::SolverType solver) {
    SolvePipeline pipeline;
    for (auto* variable : collectSolveVariables(functions, fixed, locked))
        pipeline.variableOwners.push_back(std::make_unique<Variable>(variable));
    std::vector<std::unique_ptr<::Function>> owners;
    for (const auto& binding : functions)
        if (isSolverFunction(*binding)) owners.emplace_back(binding->mathematical()->weightedFunction());
    pipeline.hasFunctions = !owners.empty();
    pipeline.hasFreeVariables = !pipeline.variableOwners.empty();
    if (!pipeline.hasFunctions || !pipeline.hasFreeVariables) return pipeline;

    std::vector<Variable*> variables;
    for (const auto& variable : pipeline.variableOwners) variables.push_back(variable.get());
    std::vector<::Function*> mathematical;
    for (auto& owner : owners) mathematical.push_back(owner.release());
    if (solver == Utils::SolverType::NEWTON)
        pipeline.task = std::make_unique<SparseSystemTask>(std::move(mathematical), std::move(variables));
    else
        pipeline.task = std::make_unique<SparseLSMTask>(std::move(mathematical), std::move(variables));
    return pipeline;
}

} // namespace OurPaintDCM::System
