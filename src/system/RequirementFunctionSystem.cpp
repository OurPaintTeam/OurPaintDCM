#include "system/RequirementFunctionSystem.h"
#include "SparseQR.h"
#include <algorithm>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Function;

RequirementFunctionSystem::RequirementFunctionSystem() = default;

void RequirementFunctionSystem::addFunction(std::shared_ptr<RequirementFunction> func) {
    _functions.push_back(func);
    for (auto v : func->getVars()) {
        if (_allVarsSet.insert(v).second) {
            _allVars.push_back(v);
        }
    }
    _jacobianDirty = true;
}

void RequirementFunctionSystem::updateJ() {
    const size_t m = _functions.size();
    const size_t n = _allVars.size();
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(m * 6);

    for (size_t i = 0; i < m; ++i) {
        auto grad = _functions[i]->gradient();
        const double weight = _functions[i]->getWeight();
        for (size_t j = 0; j < n; ++j) {
            VAR v = _allVars[j];
            double val = grad.contains(v) ? weight * grad[v] : 0.0;
            if (val != 0.0)
                triplets.emplace_back(i, j, val);
        }
    }

    _jacobian.resize(m, n);
    _jacobian.setFromTriplets(triplets.begin(), triplets.end());
    _jacobianVariableValues.clear();
    _jacobianVariableValues.reserve(n);
    for (VAR variable : _allVars) {
        _jacobianVariableValues.push_back(*variable);
    }
    _jacobianWeights.clear();
    _jacobianWeights.reserve(m);
    for (const auto& function : _functions) {
        _jacobianWeights.push_back(function->getWeight());
    }
    _jacobianDirty = false;
}

void RequirementFunctionSystem::ensureJacobian() const {
    bool needsUpdate = _jacobianDirty || _jacobianVariableValues.size() != _allVars.size() ||
                       _jacobianWeights.size() != _functions.size();
    for (size_t i = 0; !needsUpdate && i < _allVars.size(); ++i) {
        needsUpdate = *_allVars[i] != _jacobianVariableValues[i];
    }
    for (size_t i = 0; !needsUpdate && i < _functions.size(); ++i) {
        needsUpdate = _functions[i]->getWeight() != _jacobianWeights[i];
    }
    if (needsUpdate) {
        const_cast<RequirementFunctionSystem*>(this)->updateJ();
    }
}


Eigen::SparseMatrix<double> RequirementFunctionSystem::J() const {
    ensureJacobian();
    return _jacobian;
}

Eigen::SparseMatrix<double> RequirementFunctionSystem::JTJ() const {
    ensureJacobian();
    Eigen::SparseMatrix<double> JT = _jacobian.transpose();
    return JT * _jacobian;
}

Eigen::VectorXd RequirementFunctionSystem::residuals() const {
    Eigen::VectorXd r(_functions.size());
    for (size_t i = 0; i < _functions.size(); ++i)
        r[i] = _functions[i]->evaluate() * _functions[i]->getWeight();
    return r;
}

std::vector<VAR> RequirementFunctionSystem::getAllVars() const {
    return _allVars;
}

OurPaintDCM::Utils::SystemStatus RequirementFunctionSystem::diagnose() const {
    ensureJacobian();
    std::vector<std::size_t> activeRows;
    std::unordered_set<VAR> activeVars;
    for (std::size_t i = 0; i < _functions.size(); ++i) {
        if (_functions[i]->getWeight() == 0.0) {
            continue;
        }
        activeRows.push_back(i);
        for (VAR variable : _functions[i]->getVars()) {
            activeVars.insert(variable);
        }
    }
    if (activeRows.empty() || activeVars.empty()) {
        return Utils::SystemStatus::EMPTY;
    }

    std::vector<std::size_t> activeColumns;
    for (std::size_t j = 0; j < _allVars.size(); ++j) {
        if (activeVars.contains(_allVars[j])) {
            activeColumns.push_back(j);
        }
    }

    std::vector<std::size_t> rowMap(_functions.size(), _functions.size());
    for (std::size_t i = 0; i < activeRows.size(); ++i) {
        rowMap[activeRows[i]] = i;
    }

    std::vector<std::size_t> outer(activeColumns.size() + 1);
    std::vector<std::size_t> inner;
    std::vector<double> values;
    inner.reserve(_jacobian.nonZeros());
    values.reserve(_jacobian.nonZeros());
    for (std::size_t j = 0; j < activeColumns.size(); ++j) {
        outer[j] = values.size();
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 _jacobian, static_cast<Eigen::Index>(activeColumns[j])); it; ++it) {
            const auto row = rowMap[static_cast<std::size_t>(it.row())];
            if (row != _functions.size()) {
                inner.push_back(row);
                values.push_back(it.value());
            }
        }
    }
    outer.back() = values.size();

    auto activeJ = ::SparseMatrix<>::fromCSC(
        activeRows.size(), activeColumns.size(),
        std::move(values), std::move(inner), std::move(outer));
    SparseQR qr(activeJ);
    qr.setPivotThreshold(1e-8);
    qr.qr();
    const auto rank = qr.rank();
    const auto m = activeRows.size();
    const auto n = activeColumns.size();

    if (m == n && rank == n)
        return Utils::SystemStatus::WELL_CONSTRAINED;
    if (rank < std::min(m, n))
        return Utils::SystemStatus::SINGULAR_SYSTEM;
    if (m < n)
        return Utils::SystemStatus::UNDER_CONSTRAINED;
    if (m > n)
        return Utils::SystemStatus::OVER_CONSTRAINED;
    return Utils::SystemStatus::UNKNOWN;
}

void RequirementFunctionSystem::clear() {
    _functions.clear();
    _allVars.clear();
    _allVarsSet.clear();
    _jacobian.resize(0, 0);
    _jacobianVariableValues.clear();
    _jacobianWeights.clear();
    _jacobianDirty = false;
}
