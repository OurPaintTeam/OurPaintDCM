#pragma once
#include "RequirementFunction.h"
#include "tasks/matrix/SparseLSMTask.h"
#include <vector>

namespace OurPaintDCM::System {
class RequirementFunctionSystem {
    std::vector<std::shared_ptr<Function::RequirementFunction>> _functions;
    std::vector<VAR> _allVars;
    mutable std::vector<std::unique_ptr<Variable>> _variables;
    mutable std::unique_ptr<SparseLSMTask> _task;
    const SparseLSMTask& task() const;
public:
    struct Diagnosis {
        Utils::SystemStatus status;
        std::size_t variableCount;
        std::size_t constraintCount;
        std::optional<std::size_t> rank;
        std::optional<std::size_t> degreesOfFreedom;
    };

    RequirementFunctionSystem() = default;
    virtual ~RequirementFunctionSystem() = default;
    RequirementFunctionSystem(RequirementFunctionSystem&&) = default;
    RequirementFunctionSystem& operator=(RequirementFunctionSystem&&) = default;
    void addFunction(std::shared_ptr<Function::RequirementFunction> function);
    void updateJ() { task().jacobianRef(); }
    Eigen::SparseMatrix<double> J() const { return task().J(); }
    Eigen::SparseMatrix<double> JTJ() const { return task().JTJ(); }
    Eigen::VectorXd residuals() const { return task().residualVector(); }
    std::vector<VAR> getAllVars() const { return _allVars; }
    Utils::SystemStatus diagnose() const { return diagnoseDetailed().status; }
    virtual Diagnosis diagnoseDetailed() const;
    const auto& getFunctions() const { return _functions; }
    void clear() { _task.reset(); _variables.clear(); _allVars.clear(); _functions.clear(); }
protected:
    Diagnosis diagnoseWithVariables(const std::vector<VAR>& coordinates) const;
};
}
