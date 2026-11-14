#include "SolvePipeline.h"
#include "SparseLSMTask.h"
#include "SparseSystemTask.h"

namespace OurPaintDCM::System {

FixedAssignments collectFixedAssignments(RequirementBindings functions) {
    FixedAssignments assignments;
    for (const auto& binding : functions) {
        VAR variable = nullptr;
        double target = 0;
        if (binding->tryGetAssignment(variable, target)) assignments[variable] = target;
    }
    return assignments;
}

SolvePipeline buildSolvePipeline(RequirementBindings functions, const FixedAssignments& fixed,
    const std::unordered_set<VAR>& locked, Utils::SolverType solver) {
    SolvePipeline pipeline;
    std::vector<std::unique_ptr<::Function>> owners;
    std::unordered_set<VAR> seen;
    for (const auto& binding : functions) {
        if (binding->getWeight() == 0) continue;
        const auto type = binding->getType();
        if (type == Utils::RequirementType::ET_FIXPOINT || type == Utils::RequirementType::ET_FIXARC ||
            type == Utils::RequirementType::ET_FIXLINE || type == Utils::RequirementType::ET_FIXCIRCLE) continue;
        for (auto* variable : binding->getVars()) {
            if (!locked.contains(variable) && !fixed.contains(variable) && seen.insert(variable).second)
                pipeline.variableOwners.push_back(std::make_unique<Variable>(variable));
        }
        owners.emplace_back(binding->mathematical()->weightedFunction());
    }
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
