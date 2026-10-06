#pragma once
#include "RequirementFunction.h"
#include "core/ConstraintSystem.h"
#include <vector>

namespace OurPaintDCM::System {
class RequirementFunctionSystem {
    std::vector<std::shared_ptr<Function::RequirementFunction>> _functions;
    Math::ConstraintSystem _math;
public:
    RequirementFunctionSystem() = default;
    void addFunction(std::shared_ptr<Function::RequirementFunction> function) {
        if (!function) throw std::invalid_argument("Null requirement binding");
        _math.addFunction(function->mathematical());
        _functions.push_back(std::move(function));
    }
    void updateJ() { _math.updateJ(); }
    Eigen::SparseMatrix<double> J() const { return _math.J(); }
    Eigen::SparseMatrix<double> JTJ() const { return _math.JTJ(); }
    Eigen::VectorXd residuals() const { return _math.residuals(); }
    std::vector<VAR> getAllVars() const { return _math.getAllVars(); }
    Utils::SystemStatus diagnose() const;
    const auto& getFunctions() const { return _functions; }
    void clear() { _math.clear(); _functions.clear(); }
};
}
