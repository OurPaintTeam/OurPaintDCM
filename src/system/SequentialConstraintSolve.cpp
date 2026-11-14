#include "SequentialConstraintSolve.h"
#include "RequirementSystem.h"
#include <cmath>
#include <algorithm>

namespace OurPaintDCM::System {
namespace {
class ParameterState {
    std::unordered_map<VAR, double> values;
    bool accepted = false;
public:
    explicit ParameterState(const RequirementSystem& system) {
        for (const auto& function : system.getFunctions())
            for (auto* variable : function->getVars()) values.try_emplace(variable, *variable);
    }
    void accept() { accepted = true; }
    ~ParameterState() {
        if (!accepted) for (const auto& [variable, value] : values) *variable = value;
    }
};
}

bool solveSequentialConstraints(const RequirementSystem& system, const ConstraintDecomposition& plan,
    std::span<const std::size_t> order, const std::unordered_set<VAR>& locked,
    Utils::SolverType solver, double tolerance, const std::function<void(TaskMatrix&)>& optimize) {
    ParameterState original(system);
    const auto fixed = collectFixedAssignments(system.getFunctions());
    auto determined = locked;
    for (const auto& [variable, target] : fixed) {
        if (locked.contains(variable) && *variable != target) return false;
        *variable = target;
    }
    for (auto index : order) {
        std::vector<std::shared_ptr<Function::RequirementFunction>> bindings;
        for (auto function : plan.blocks.at(index).functionIndices)
            bindings.push_back(system.getFunctions().at(function));
        for (const auto& binding : bindings)
            if (!std::isfinite(binding->mathematical()->weightedValue())) return false;
        auto pipeline = buildSolvePipeline(bindings, fixed, determined, solver);
        if (pipeline.task) optimize(*pipeline.task);
        for (const auto& binding : bindings) {
            if (!binding->mathematical()->satisfied(tolerance)) return false;
            if (binding->getWeight() == 0) continue;
            for (auto* variable : binding->getVars())
                if (std::binary_search(plan.sharedVariables.begin(), plan.sharedVariables.end(),
                                       variable, std::less<VAR>{})) determined.insert(variable);
        }
    }
    for (const auto& binding : system.getFunctions())
        if (!binding->mathematical()->satisfied(tolerance)) return false;
    original.accept();
    return true;
}

} // namespace OurPaintDCM::System
