#pragma once
#include "core/Constraint.h"
#include "Enums.h"
#include "ID.h"
#include <memory>
#include <optional>

using VAR = double*;

namespace OurPaintDCM::Function {
// Binding metadata only. Values, derivatives, weights and assignment equations
// belong to math. Geometry must outlive the binding and every mathematical clone.
class RequirementFunction {
    Utils::RequirementType _type;
    std::shared_ptr<Math::Constraint> _math;
    std::optional<Utils::ID> _requirementId;
public:
    RequirementFunction(Utils::RequirementType type, std::shared_ptr<Math::Constraint> function)
        : _type(type), _math(std::move(function)) {
        if (!_math) throw std::invalid_argument("Null mathematical constraint");
    }
    const std::shared_ptr<Math::Constraint>& mathematical() const { return _math; }
    Utils::RequirementType getType() const { return _type; }
    const std::vector<VAR>& getVars() const { return _math->variables(); }
    size_t getVarCount() const { return getVars().size(); }
    double getWeight() const { return _math->weight(); }
    void setWeight(double weight) { _math->setWeight(weight); }
    bool tryGetAssignment(VAR& variable, double& target) const {
        return getWeight() != 0 && _math->assignment(variable,target);
    }
    void setRequirementId(Utils::ID id) { _requirementId = id; }
    std::optional<Utils::ID> requirementId() const { return _requirementId; }
};
}
