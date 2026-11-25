#include "ConstraintRankDiagnosis.h"
#include "SparseQR.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace OurPaintDCM::System {
namespace {

bool validLinearization(const ConstraintLinearization& input) {
    if (!input.defined || input.jacobian.rowMutableMode()
        || input.jacobian.rows_size() != input.equations.size()
        || input.jacobian.cols_size() != input.variables.size()
        || input.residuals.rows_size() != input.equations.size() || input.residuals.cols_size() != 1) return false;
    for (std::size_t row = 0; row < input.equations.size(); ++row)
        if (!std::isfinite(input.residuals(row, 0)) || !std::isfinite(input.equations[row].weight)) return false;
    for (double value : input.jacobian.valueData())
        if (!std::isfinite(value)) return false;
    return true;
}

bool normalizeColumn(SparseMatrix<>& matrix, std::size_t column) {
    const auto& outer = matrix.outerIndexData();
    auto& values = matrix.mutableValueData();
    double scale = 0;
    for (auto index = outer[column]; index < outer[column + 1]; ++index)
        scale = std::max(scale, std::abs(values[index]));
    if (scale == 0) return true;
    for (auto index = outer[column]; index < outer[column + 1]; ++index) {
        const double value = values[index] / scale;
        if (values[index] != 0 && value == 0) return false;
        values[index] = value;
    }
    return true;
}

std::optional<SparseMatrix<>> equilibratedTranspose(const ConstraintLinearization& input) {
    auto result = input.jacobian.transpose();
    std::vector<double> coordinateScales(result.rows_size(), 0);
    const auto& outer = result.outerIndexData();
    const auto& inner = result.innerIndexData();
    auto& values = result.mutableValueData();
    for (std::size_t column = 0; column < result.cols_size(); ++column) {
        if (input.equations[column].weight == 0) {
            std::fill(values.begin() + outer[column], values.begin() + outer[column + 1], 0);
            continue;
        }
        if (!normalizeColumn(result, column)) return std::nullopt;
        for (auto index = outer[column]; index < outer[column + 1]; ++index)
            coordinateScales[inner[index]] = std::max(coordinateScales[inner[index]], std::abs(values[index]));
    }
    for (std::size_t column = 0; column < result.cols_size(); ++column) {
        for (auto index = outer[column]; index < outer[column + 1]; ++index)
            if (coordinateScales[inner[index]] != 0) values[index] /= coordinateScales[inner[index]];
        if (!normalizeColumn(result, column)) return std::nullopt;
    }
    result.prune();
    return result;
}

double largestColumnNorm(const SparseMatrix<>& matrix) {
    double largest = 0;
    for (std::size_t column = 0; column < matrix.cols_size(); ++column) {
        double norm = 0;
        for (SparseMatrix<>::InnerIterator it(matrix, column); it; ++it) norm = std::hypot(norm, it.value());
        largest = std::max(largest, norm);
    }
    return largest;
}

} // namespace

ConstraintRankDiagnosis diagnoseConstraintRank(const ConstraintLinearization& input, double relativeTolerance) {
    if (!std::isfinite(relativeTolerance) || relativeTolerance <= 0 || relativeTolerance >= 1)
        throw std::invalid_argument("Rank tolerance must be finite and between zero and one");
    if (!validLinearization(input)) return {};
    const auto scaled = equilibratedTranspose(input);
    if (!scaled) return {};

    ConstraintRankDiagnosis result;
    result.status = ConstraintRankDiagnosis::Status::DEFINED;
    result.rank = 0;
    const double scale = largestColumnNorm(*scaled);
    if (scale == 0) {
        for (std::size_t row = 0; row < input.equations.size(); ++row)
            if (input.equations[row].weight != 0) result.dependentEquations.push_back(row);
        return result;
    }

    SparseQR qr(*scaled);
    qr.setPivotThreshold(relativeTolerance * scale);
    qr.qrRankRevealing();
    result.rank = qr.rank();
    for (std::size_t column = 0; column < qr.colsPermutation().size(); ++column) {
        const auto row = qr.colsPermutation()[column];
        if (input.equations[row].weight == 0) continue;
        auto& equations = column < *result.rank ? result.independentEquations : result.dependentEquations;
        equations.push_back(row);
    }
    return result;
}

} // namespace OurPaintDCM::System
