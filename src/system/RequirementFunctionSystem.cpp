#include "RequirementFunctionSystem.h"
#include <algorithm>
#include <cmath>

using namespace OurPaintDCM::System;

void RequirementFunctionSystem::addFunction(std::shared_ptr<OurPaintDCM::Function::RequirementFunction> function) {
    if (!function) throw std::invalid_argument("Null requirement binding");
    _task.reset();
    _variables.clear();
    for (auto* variable : function->getVars())
        if (std::find(_allVars.begin(),_allVars.end(),variable) == _allVars.end()) _allVars.push_back(variable);
    _functions.push_back(std::move(function));
}

const SparseLSMTask& RequirementFunctionSystem::task() const {
    if (!_task) {
        _variables.clear();
        std::vector<Variable*> variables;
        for (auto* coordinate : _allVars) {
            _variables.push_back(std::make_unique<Variable>(coordinate));
            variables.push_back(_variables.back().get());
        }
        std::vector<std::unique_ptr<::Function>> owners;
        for (const auto& binding : _functions) owners.emplace_back(binding->mathematical()->weightedFunction());
        std::vector<::Function*> functions;
        functions.reserve(owners.size());
        for (auto& owner : owners) functions.push_back(owner.release());
        _task = std::make_unique<SparseLSMTask>(std::move(functions),std::move(variables));
    }
    return *_task;
}

ConstraintLinearization RequirementFunctionSystem::linearizeConstraints() const {
    ConstraintLinearization result;
    result.variables = _allVars;
    result.equations.reserve(_functions.size());
    for (std::size_t i = 0; i < _functions.size(); ++i) {
        const auto& binding = _functions[i];
        result.equations.push_back({i, binding->requirementId(), binding->getWeight()});
    }
    try {
        const auto [residuals, jacobian] = task().linearizationView();
        result.jacobian = jacobian.get();
        result.residuals = residuals.get();
    } catch (const FunctionDomainError&) {
        return result;
    } catch (const std::domain_error&) {
        return result;
    }
    for (std::size_t row = 0; row < result.residuals.rows_size(); ++row)
        if (!std::isfinite(result.residuals(row, 0))) return result;
    for (double value : result.jacobian.valueData())
        if (!std::isfinite(value)) return result;
    result.defined = true;
    return result;
}

namespace {
OurPaintDCM::Utils::SystemStatus convertStatus(SparseLSMTask::DiagnosticStatus status) {
    using Status = SparseLSMTask::DiagnosticStatus;
    switch (status) {
        case Status::EMPTY: return OurPaintDCM::Utils::SystemStatus::EMPTY;
        case Status::WELL_CONSTRAINED: return OurPaintDCM::Utils::SystemStatus::WELL_CONSTRAINED;
        case Status::UNDER_CONSTRAINED: return OurPaintDCM::Utils::SystemStatus::UNDER_CONSTRAINED;
        case Status::OVER_CONSTRAINED: return OurPaintDCM::Utils::SystemStatus::OVER_CONSTRAINED;
        case Status::SINGULAR_SYSTEM: return OurPaintDCM::Utils::SystemStatus::SINGULAR_SYSTEM;
        default: return OurPaintDCM::Utils::SystemStatus::UNKNOWN;
    }
}

RequirementFunctionSystem::Diagnosis convertDiagnosis(const SparseLSMTask::Diagnosis& result) {
    return {convertStatus(result.status), result.variableCount, result.constraintCount,
            result.rank, result.degreesOfFreedom};
}
} // namespace

RequirementFunctionSystem::Diagnosis RequirementFunctionSystem::diagnoseDetailed() const {
    return convertDiagnosis(task().diagnoseDetailed());
}

RequirementFunctionSystem::Diagnosis RequirementFunctionSystem::diagnoseWithVariables(
    const std::vector<VAR>& coordinates) const {
    std::vector<std::unique_ptr<Variable>> owners;
    std::vector<Variable*> variables;
    for (auto* coordinate : coordinates) {
        owners.push_back(std::make_unique<Variable>(coordinate));
        variables.push_back(owners.back().get());
    }
    std::vector<std::unique_ptr<::Function>> functionOwners;
    for (const auto& binding : _functions)
        functionOwners.emplace_back(binding->mathematical()->weightedFunction());
    std::vector<::Function*> functions;
    functions.reserve(functionOwners.size());
    for (auto& owner : functionOwners) functions.push_back(owner.release());
    SparseLSMTask diagnosticTask(std::move(functions), std::move(variables));
    return convertDiagnosis(diagnosticTask.diagnoseDetailed(SparseLSMTask::DiagnosticScope::ALL_VARIABLES));
}
