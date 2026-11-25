#include "RequirementFunctionFactory.h"
#include "RequirementSystem.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Function;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {
class CountedAssignment : public FixCoordinateError {
    std::size_t& _copies;
public:
    CountedAssignment(VAR coordinate, std::size_t& copies)
        : FixCoordinateError(std::vector<VAR>{coordinate}, 0), _copies(copies) {}
    ::Function* weightedFunction() const override {
        ++_copies;
        return FixCoordinateError::weightedFunction();
    }
};
} // namespace

TEST(ConstraintLinearizationTest, ReusesTheExistingSymbolicTaskAcrossSnapshots) {
    RequirementFunctionSystem system;
    double x = 2;
    std::size_t copies = 0;
    system.addFunction(std::make_shared<RequirementFunction>(RequirementType::ET_FIXPOINT,
        std::make_shared<CountedAssignment>(&x, copies)));
    EXPECT_TRUE(system.linearizeConstraints().defined);
    EXPECT_EQ(copies, 1U);
    x = 3;
    EXPECT_DOUBLE_EQ(system.linearizeConstraints().residuals(0, 0), 3);
    EXPECT_EQ(copies, 1U);
}

TEST(ConstraintLinearizationTest, RetainsWeightsOriginsAndCachedValuesWithoutAssigningCoordinates) {
    RequirementFunctionSystem system;
    double x = 2;
    auto first = RequirementFunctionFactory::bind<FixCoordinateError>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&x}, 0);
    first->setWeight(3);
    first->setRequirementId(ID{17});
    auto second = RequirementFunctionFactory::bind<FixCoordinateError>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&x}, 4);
    second->setWeight(0);
    system.addFunction(first);
    system.addFunction(second);

    const auto snapshot = system.linearizeConstraints();
    ASSERT_TRUE(snapshot.defined);
    ASSERT_EQ(snapshot.equations.size(), 2U);
    EXPECT_EQ(snapshot.variables, (std::vector<VAR>{&x}));
    EXPECT_EQ(snapshot.equations[0].functionIndex, 0U);
    EXPECT_EQ(snapshot.equations[0].requirementId, ID{17});
    EXPECT_DOUBLE_EQ(snapshot.equations[0].weight, 3);
    EXPECT_EQ(snapshot.equations[1].functionIndex, 1U);
    EXPECT_FALSE(snapshot.equations[1].requirementId);
    EXPECT_DOUBLE_EQ(snapshot.equations[1].weight, 0);
    EXPECT_DOUBLE_EQ(snapshot.jacobian(0, 0), 3);
    EXPECT_DOUBLE_EQ(snapshot.residuals(0, 0), 6);
    EXPECT_DOUBLE_EQ(snapshot.residuals(1, 0), 0);
    EXPECT_DOUBLE_EQ(x, 2);

    x = 5;
    first->setWeight(2);
    const auto refreshed = system.linearizeConstraints();
    ASSERT_TRUE(refreshed.defined);
    EXPECT_DOUBLE_EQ(refreshed.residuals(0, 0), 10);
    EXPECT_DOUBLE_EQ(refreshed.jacobian(0, 0), 2);
    EXPECT_DOUBLE_EQ(snapshot.residuals(0, 0), 6);
    EXPECT_DOUBLE_EQ(snapshot.jacobian(0, 0), 3);
    EXPECT_DOUBLE_EQ(x, 5);
}

TEST(ConstraintLinearizationTest, PreservesMultipleEquationsOfOneGeometricRequirement) {
    GeometryStorage storage;
    const auto point = storage.createPoint(2, 3);
    RequirementSystem system(&storage);
    const auto requirement = system.addRequirement(RequirementDescriptor::fixPoint(point));
    const auto snapshot = system.linearizeConstraints();
    ASSERT_TRUE(snapshot.defined);
    ASSERT_EQ(snapshot.equations.size(), 2U);
    EXPECT_EQ(snapshot.equations[0].requirementId, requirement);
    EXPECT_EQ(snapshot.equations[1].requirementId, requirement);
    EXPECT_NE(snapshot.equations[0].functionIndex, snapshot.equations[1].functionIndex);
    EXPECT_EQ(snapshot.jacobian.rows_size(), 2);
    EXPECT_EQ(snapshot.jacobian.cols_size(), 2);
}

TEST(ConstraintLinearizationTest, ImplicitArcEquationHasNoUserRequirementId) {
    GeometryStorage storage;
    const auto a = storage.createPoint(0, 0);
    const auto b = storage.createPoint(2, 0);
    const auto center = storage.createPoint(1, 1);
    ASSERT_TRUE(storage.createArc(a, b, center));
    RequirementSystem system(&storage);
    const auto snapshot = system.linearizeConstraints();
    ASSERT_TRUE(snapshot.defined);
    ASSERT_EQ(snapshot.equations.size(), 1U);
    EXPECT_FALSE(snapshot.equations[0].requirementId);
    EXPECT_EQ(snapshot.equations[0].functionIndex, 0U);
}

TEST(ConstraintLinearizationTest, UndefinedGeometryRetainsOriginsAndDoesNotMovePoints) {
    GeometryStorage storage;
    const auto first = storage.createPoint(0, 0);
    const auto second = storage.createPoint(1, 0);
    const auto line = storage.createLine(first, second);
    ASSERT_TRUE(line);
    RequirementSystem system(&storage);
    const auto requirement = system.addRequirement(RequirementDescriptor::horizontal(*line));
    storage.get<Point2D>(second)->x() = 0;
    const auto snapshot = system.linearizeConstraints();
    EXPECT_FALSE(snapshot.defined);
    ASSERT_EQ(snapshot.equations.size(), 1U);
    EXPECT_EQ(snapshot.equations[0].requirementId, requirement);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(second)->x(), 0);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(second)->y(), 0);
}

TEST(ConstraintLinearizationTest, NonFiniteResidualIsUndefinedAndEmptySystemIsDefined) {
    RequirementFunctionSystem system;
    EXPECT_TRUE(system.linearizeConstraints().defined);
    double x = 0;
    system.addFunction(RequirementFunctionFactory::bind<FixCoordinateError>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&x}, 0));
    x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(system.linearizeConstraints().defined);
    system.clear();
    const auto empty = system.linearizeConstraints();
    EXPECT_TRUE(empty.defined);
    EXPECT_EQ(empty.jacobian.rows_size(), 0);
    EXPECT_EQ(empty.jacobian.cols_size(), 0);
    EXPECT_TRUE(empty.equations.empty());
    EXPECT_TRUE(empty.variables.empty());
}
