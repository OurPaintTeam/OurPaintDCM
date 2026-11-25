#include "ConstraintRankDiagnosis.h"
#include "RequirementSystem.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {

ConstraintLinearization linearization(const Matrix<>& jacobian) {
    ConstraintLinearization result;
    result.jacobian = SparseMatrix<>(jacobian);
    result.residuals = Matrix<>(jacobian.rows_size(), 1);
    result.variables.resize(jacobian.cols_size());
    for (std::size_t row = 0; row < jacobian.rows_size(); ++row)
        result.equations.push_back({row, std::nullopt, 1});
    result.defined = true;
    return result;
}

// Small-test oracle using elimination, independent of QR's basis selection.
std::size_t denseRank(Matrix<> matrix) {
    std::size_t rank = 0;
    for (std::size_t column = 0; column < matrix.cols_size() && rank < matrix.rows_size(); ++column) {
        std::size_t pivot = rank;
        for (std::size_t row = rank + 1; row < matrix.rows_size(); ++row)
            if (std::abs(matrix(row, column)) > std::abs(matrix(pivot, column))) pivot = row;
        if (std::abs(matrix(pivot, column)) < 1e-10) continue;
        for (std::size_t next = column; next < matrix.cols_size(); ++next)
            std::swap(matrix(rank, next), matrix(pivot, next));
        for (std::size_t row = rank + 1; row < matrix.rows_size(); ++row) {
            const double factor = matrix(row, column) / matrix(rank, column);
            for (std::size_t next = column; next < matrix.cols_size(); ++next)
                matrix(row, next) -= factor * matrix(rank, next);
        }
        ++rank;
    }
    return rank;
}

void expectBasis(const ConstraintLinearization& input, std::size_t rank) {
    const auto result = diagnoseConstraintRank(input);
    ASSERT_EQ(result.status, ConstraintRankDiagnosis::Status::DEFINED);
    ASSERT_EQ(result.rank, rank);
    ASSERT_EQ(result.independentEquations.size(), rank);
    auto rows = result.independentEquations;
    rows.insert(rows.end(), result.dependentEquations.begin(), result.dependentEquations.end());
    std::sort(rows.begin(), rows.end());
    std::vector<std::size_t> active;
    for (std::size_t row = 0; row < input.equations.size(); ++row)
        if (input.equations[row].weight != 0) active.push_back(row);
    EXPECT_EQ(rows, active);
    Matrix<> basis(rank + 1, input.jacobian.cols_size());
    for (std::size_t row = 0; row < rank; ++row)
        for (std::size_t column = 0; column < basis.cols_size(); ++column)
            basis(row, column) = input.jacobian(result.independentEquations[row], column);
    EXPECT_EQ(denseRank(basis), rank);
    for (auto row : result.dependentEquations) {
        for (std::size_t column = 0; column < basis.cols_size(); ++column)
            basis(rank, column) = input.jacobian(row, column);
        EXPECT_EQ(denseRank(basis), rank);
    }
}

TEST(ConstraintRankDiagnosisTest, IndependentRectangularSystems) {
    Matrix<> wide{{1, 0, 2, 0}, {0, 1, 0, 3}};
    expectBasis(linearization(wide), 2);
    expectBasis(linearization(wide.transpose()), 2);
}

TEST(ConstraintRankDiagnosisTest, LateIndependentEquationSurvivesDependentLeadingColumns) {
    expectBasis(linearization({{1, 1}, {1, 1}, {1, 2}}), 2);
}

TEST(ConstraintRankDiagnosisTest, DuplicatesAndTransitiveDependenciesPreserveAValidBasisInEveryOrder) {
    Matrix<> jacobian{{1, 0, 0}, {2, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 0}};
    std::vector<int> order{0, 1, 2, 3, 4};
    do {
        Matrix<> permuted(5, 3);
        for (std::size_t row = 0; row < order.size(); ++row)
            for (std::size_t column = 0; column < 3; ++column)
                permuted(row, column) = jacobian(order[row], column);
        expectBasis(linearization(permuted), 2);
    } while (std::next_permutation(order.begin(), order.end()));
}

TEST(ConstraintRankDiagnosisTest, ZeroRankEmptyDimensionsAndZeroWeightEquations) {
    auto zero = linearization(Matrix<>(3, 2));
    zero.equations[1].weight = 0;
    const auto result = diagnoseConstraintRank(zero);
    ASSERT_EQ(result.rank, 0U);
    EXPECT_TRUE(result.independentEquations.empty());
    EXPECT_EQ(result.dependentEquations, (std::vector<std::size_t>{0, 2}));
    EXPECT_EQ(diagnoseConstraintRank(linearization(Matrix<>(0, 3))).rank, 0U);
    EXPECT_EQ(diagnoseConstraintRank(linearization(Matrix<>(3, 0))).rank, 0U);
    auto disabled = linearization(Matrix<>::identity(2, 2));
    disabled.equations[0].weight = 0;
    const auto active = diagnoseConstraintRank(disabled);
    ASSERT_EQ(active.rank, 1U);
    EXPECT_EQ(active.independentEquations, (std::vector<std::size_t>{1}));
    EXPECT_TRUE(active.dependentEquations.empty());
}

TEST(ConstraintRankDiagnosisTest, EquivalentEquationAndCoordinateScalesPreserveRank) {
    const Matrix<> jacobian{{1, 1}, {1, 2}, {2, 3}};
    for (double rowScale : {1e-100, 1.0, 1e100}) {
        for (double coordinateScale : {1e-100, 1.0, 1e100}) {
            auto scaled = jacobian;
            for (std::size_t column = 0; column < 2; ++column) {
                scaled(0, column) *= rowScale;
                scaled(2, column) *= 3 * rowScale;
            }
            for (std::size_t row = 0; row < 3; ++row) scaled(row, 1) *= coordinateScale;
            const auto result = diagnoseConstraintRank(linearization(scaled));
            EXPECT_EQ(result.rank, 2U);
            EXPECT_EQ(result.independentEquations.size(), 2U);
            EXPECT_EQ(result.dependentEquations.size(), 1U);
        }
    }
}

TEST(ConstraintRankDiagnosisTest, NearDependenceUsesRelativeTolerance) {
    const auto input = linearization({{1, 1}, {1, 1 + 1e-10}});
    EXPECT_EQ(diagnoseConstraintRank(input, 1e-8).rank, 1U);
    EXPECT_EQ(diagnoseConstraintRank(input, 1e-12).rank, 2U);
    EXPECT_THROW(diagnoseConstraintRank(input, 0), std::invalid_argument);
    EXPECT_THROW(diagnoseConstraintRank(input, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(ConstraintRankDiagnosisTest, UndefinedOrNonFiniteInputHasNoBasis) {
    auto input = linearization(Matrix<>::identity(2, 2));
    input.defined = false;
    EXPECT_FALSE(diagnoseConstraintRank(input).rank);
    input.defined = true;
    input.jacobian.coeffRef(0, 0) = std::numeric_limits<double>::infinity();
    const auto invalid = diagnoseConstraintRank(input);
    EXPECT_EQ(invalid.status, ConstraintRankDiagnosis::Status::NUMERICALLY_UNDEFINED);
    EXPECT_TRUE(invalid.independentEquations.empty());
    EXPECT_TRUE(invalid.dependentEquations.empty());
    input.jacobian = SparseMatrix<>::identity(2);
    input.residuals(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(diagnoseConstraintRank(input).rank);
}

TEST(ConstraintRankDiagnosisTest, EquilibrationUnderflowIsNumericallyUndefined) {
    const auto result = diagnoseConstraintRank(linearization({{1e300, 1e-300}, {1e300, 2e-300}}));
    EXPECT_EQ(result.status, ConstraintRankDiagnosis::Status::NUMERICALLY_UNDEFINED);
    EXPECT_FALSE(result.rank);
}

TEST(ConstraintRankDiagnosisTest, LocalDifferentialDependenceDoesNotClassifyNonlinearCompatibility) {
    // x = 0, x + y^2 - 1 = 0 at (0,0): globally solvable, locally dependent.
    auto input = linearization({{1, 0}, {1, 0}});
    input.residuals(1, 0) = -1;
    expectBasis(input, 1);
    EXPECT_DOUBLE_EQ(input.residuals(1, 0), -1);
    input.residuals(1, 0) = 0;
    expectBasis(input, 1);
}

TEST(ConstraintRankDiagnosisTest, GeometricDuplicateDistancesRetainOriginalRequirementIds) {
    GeometryStorage storage;
    const auto a = storage.createPoint(0, 0);
    const auto b = storage.createPoint(3, 4);
    RequirementSystem system(&storage);
    const auto first = system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    auto descriptor = RequirementDescriptor::pointPointDist(a, b, 7);
    descriptor.weight = 1e-100;
    const auto second = system.addRequirement(descriptor);
    const auto snapshot = system.linearizeConstraints();
    const auto result = diagnoseConstraintRank(snapshot);
    ASSERT_EQ(result.rank, 1U);
    ASSERT_EQ(result.independentEquations.size(), 1U);
    ASSERT_EQ(result.dependentEquations.size(), 1U);
    std::vector<ID> requirements;
    for (auto row : {result.independentEquations.front(), result.dependentEquations.front()}) {
        ASSERT_TRUE(snapshot.equations[row].requirementId);
        requirements.push_back(*snapshot.equations[row].requirementId);
    }
    std::sort(requirements.begin(), requirements.end());
    EXPECT_EQ(requirements, (std::vector<ID>{first, second}));
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->x(), 3);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->y(), 4);
}

} // namespace
