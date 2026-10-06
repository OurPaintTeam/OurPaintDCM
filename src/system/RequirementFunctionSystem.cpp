#include "RequirementFunctionSystem.h"
#include <algorithm>

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

OurPaintDCM::Utils::SystemStatus RequirementFunctionSystem::diagnose() const {
    using Status = SparseLSMTask::DiagnosticStatus;
    switch (task().diagnose()) {
        case Status::EMPTY: return OurPaintDCM::Utils::SystemStatus::EMPTY;
        case Status::WELL_CONSTRAINED: return OurPaintDCM::Utils::SystemStatus::WELL_CONSTRAINED;
        case Status::UNDER_CONSTRAINED: return OurPaintDCM::Utils::SystemStatus::UNDER_CONSTRAINED;
        case Status::OVER_CONSTRAINED: return OurPaintDCM::Utils::SystemStatus::OVER_CONSTRAINED;
        case Status::SINGULAR_SYSTEM: return OurPaintDCM::Utils::SystemStatus::SINGULAR_SYSTEM;
        default: return OurPaintDCM::Utils::SystemStatus::UNKNOWN;
    }
}
