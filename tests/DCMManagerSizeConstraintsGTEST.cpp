#include "DCMManager.h"
#include "RequirementFunctionFactory.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
constexpr SolveMode modes[] = {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG};

bool solve(DCMManager& manager, ID figure) {
    return manager.solve(manager.getComponentForFigure(figure));
}

double length(const DCMManager& manager, ID line) {
    const auto d = manager.getFigure(line).value();
    return std::hypot(d.coords[2] - d.coords[0], d.coords[3] - d.coords[1]);
}

double pointRadius(const DCMManager& manager, ID point, ID circle) {
    const auto p = manager.getFigure(point).value();
    const auto c = manager.getFigure(circle).value();
    return std::hypot(*p.x - c.coords[0], *p.y - c.coords[1]);
}

void expectFreedom(const DCMManager& manager, std::size_t degrees) {
    const auto report = manager.getRequirementSystem().diagnoseDetailed();
    EXPECT_EQ(report.status, SystemStatus::UNDER_CONSTRAINED);
    EXPECT_EQ(report.degreesOfFreedom, degrees);
}

void expectPoint(const DCMManager& manager, ID point, double x, double y) {
    const auto d = manager.getFigure(point).value();
    EXPECT_DOUBLE_EQ(*d.x, x);
    EXPECT_DOUBLE_EQ(*d.y, y);
}
}

TEST(DCMManagerSizeConstraints, PointOnCircleSolvesFromCenterAndOutsideWithFixedCircle) {
    for (auto mode : modes) for (bool atCenter : {false, true}) {
        SCOPED_TRACE(static_cast<int>(mode));
        SCOPED_TRACE(atCenter);
        DCMManager manager;
        auto circle = manager.addFigure(FigureDescriptor::circle(2, 3, 5));
        auto point = manager.addFigure(FigureDescriptor::point(atCenter ? 2 : 9, atCenter ? 3 : 8));
        manager.addRequirement(RequirementDescriptor::fixCircle(circle));
        auto on = manager.addRequirement(RequirementDescriptor::pointOnCircle(point, circle));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, point));
        EXPECT_NEAR(pointRadius(manager, point, circle), 5, 1e-6);
        const auto c = manager.getFigure(circle).value();
        EXPECT_DOUBLE_EQ(c.coords[0], 2);
        EXPECT_DOUBLE_EQ(c.coords[1], 3);
        EXPECT_DOUBLE_EQ(*c.radius, 5);
        expectFreedom(manager, 1);
        ASSERT_TRUE(solve(manager, point));
        const auto& rows = manager.getRequirementSystem().getFunctions();
        EXPECT_EQ(rows.back()->requirementId(), on);
        EXPECT_NE(dynamic_cast<PointOnCircleError*>(rows.back()->mathematical().get()), nullptr);
    }
}

TEST(DCMManagerSizeConstraints, PointOnCircleUsesCoincidentPointAndCenterRepresentatives) {
    for (auto mode : modes) {
        DCMManager manager;
        auto center = manager.addFigure(FigureDescriptor::point(0, 0));
        auto centerAlias = manager.addFigure(FigureDescriptor::point(1, 2));
        auto circle = manager.addFigure(FigureDescriptor::circle(center, 4));
        auto point = manager.addFigure(FigureDescriptor::point(8, 2));
        auto pointAlias = manager.addFigure(FigureDescriptor::point(9, 2));
        manager.addRequirement(RequirementDescriptor::pointOnPoint(center, centerAlias));
        manager.addRequirement(RequirementDescriptor::pointOnPoint(point, pointAlias));
        manager.addRequirement(RequirementDescriptor::fixCircle(circle));
        manager.addRequirement(RequirementDescriptor::pointOnCircle(point, circle));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, point));
        EXPECT_NEAR(pointRadius(manager, point, circle), 4, 1e-6);
        const auto p = manager.getFigure(point).value();
        expectPoint(manager, pointAlias, *p.x, *p.y);
        expectPoint(manager, center, 1, 2);
        expectFreedom(manager, 1);
    }
}

TEST(DCMManagerSizeConstraints, PointOnCircleWeightsRemovalSnapshotsAndDragInvalidateState) {
    for (auto mode : modes) {
        DCMManager manager;
        auto circle = manager.addFigure(FigureDescriptor::circle(0, 0, 5));
        auto point = manager.addFigure(FigureDescriptor::point(3, 4));
        manager.addRequirement(RequirementDescriptor::fixCircle(circle));
        auto on = manager.addRequirement(RequirementDescriptor::pointOnCircle(point, circle));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, point));
        const auto saved = manager.snapshot();
        manager.updateRequirementWeight(on, 0);
        manager.updatePoint({point, 8, 0});
        ASSERT_TRUE(solve(manager, point));
        expectPoint(manager, point, 8, 0);
        expectFreedom(manager, 2);
        manager.updateRequirementWeight(on, 2);
        ASSERT_TRUE(solve(manager, point));
        EXPECT_NEAR(pointRadius(manager, point, circle), 5, 1e-6);
        manager.removeRequirement(on);
        EXPECT_NE(manager.getComponentForFigure(point), manager.getComponentForFigure(circle));
        manager.updatePoint({point, 1, 0});
        ASSERT_TRUE(solve(manager, point));
        expectPoint(manager, point, 1, 0);
        manager.restoreSnapshot(saved);
        EXPECT_EQ(manager.getRequirement(on)->type, RequirementType::ET_POINTONCIRCLE);
        ASSERT_TRUE(solve(manager, point));
        expectFreedom(manager, 1);
        if (mode == SolveMode::DRAG) {
            manager.updatePoint({point, 0, 0});
            EXPECT_NEAR(pointRadius(manager, point, circle), 5, 1e-6);
        }
        manager.removeFigure(circle, true);
        EXPECT_FALSE(manager.hasRequirement(on));
        expectFreedom(manager, 2);
    }
}

TEST(DCMManagerSizeConstraints, RadiusAndDiameterOnlyConstrainRadiusInEveryMode) {
    for (auto mode : modes) for (bool diameter : {false, true}) {
        DCMManager manager;
        auto circle = manager.addFigure(FigureDescriptor::circle(3, 4, 2));
        auto size = manager.addRequirement(diameter ? RequirementDescriptor::circleDiameter(circle, 20)
                                                   : RequirementDescriptor::circleRadius(circle, 10));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_NEAR(*manager.getFigure(circle)->radius, 10, 1e-6);
        EXPECT_DOUBLE_EQ(manager.getFigure(circle)->coords[0], 3);
        EXPECT_DOUBLE_EQ(manager.getFigure(circle)->coords[1], 4);
        expectFreedom(manager, 2);
        const auto& row = manager.getRequirementSystem().getFunctions().front();
        EXPECT_EQ(row->getVars().size(), 1u);
        EXPECT_EQ(row->requirementId(), size);
        EXPECT_NE(dynamic_cast<CircleRadiusError*>(row->mathematical().get()), nullptr);
        manager.updateCircle(CircleUpdateDescriptor::center(circle, 6, 8));
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_DOUBLE_EQ(manager.getFigure(circle)->coords[0], 6);
        EXPECT_DOUBLE_EQ(manager.getFigure(circle)->coords[1], 8);
        EXPECT_NEAR(*manager.getFigure(circle)->radius, 10, 1e-6);
    }
}

TEST(DCMManagerSizeConstraints, SizeParametersWeightsRemovalAndSnapshotsPreserveOriginalUnits) {
    for (auto mode : modes) for (bool diameter : {false, true}) {
        DCMManager manager;
        auto circle = manager.addFigure(FigureDescriptor::circle(1, 2, 1));
        auto size = manager.addRequirement(diameter ? RequirementDescriptor::circleDiameter(circle, 6)
                                                   : RequirementDescriptor::circleRadius(circle, 3));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, circle));
        const auto saved = manager.snapshot();
        manager.updateRequirementParam(size, diameter ? 20 : 10);
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_NEAR(*manager.getFigure(circle)->radius, 10, 1e-6);
        EXPECT_DOUBLE_EQ(*manager.getRequirement(size)->param, diameter ? 20 : 10);
        manager.updateRequirementWeight(size, 0);
        manager.updateCircle({circle, 7});
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_DOUBLE_EQ(*manager.getFigure(circle)->radius, 7);
        expectFreedom(manager, 3);
        manager.updateRequirementWeight(size, 2);
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_NEAR(*manager.getFigure(circle)->radius, 10, 1e-6);
        manager.removeRequirement(size);
        manager.updateCircle({circle, 8});
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_DOUBLE_EQ(*manager.getFigure(circle)->radius, 8);
        manager.restoreSnapshot(saved);
        EXPECT_EQ(manager.getRequirement(size)->type, diameter ? RequirementType::ET_CIRCLEDIAMETER : RequirementType::ET_CIRCLERADIUS);
        EXPECT_DOUBLE_EQ(*manager.getRequirement(size)->param, diameter ? 6 : 3);
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_NEAR(*manager.getFigure(circle)->radius, 3, 1e-6);
        expectFreedom(manager, 2);
        manager.removeFigure(circle, true);
        EXPECT_FALSE(manager.hasRequirement(size));
        EXPECT_EQ(manager.getRequirementSystem().diagnose(), SystemStatus::EMPTY);
    }
}

TEST(DCMManagerSizeConstraints, EquivalentSizesAgreeAndConflictingOrFixedSizesFail) {
    for (auto mode : modes) {
        DCMManager manager;
        auto circle = manager.addFigure(FigureDescriptor::circle(1, 2, 3));
        manager.addRequirement(RequirementDescriptor::circleRadius(circle, 10));
        auto diameter = manager.addRequirement(RequirementDescriptor::circleDiameter(circle, 20));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, circle));
        EXPECT_NEAR(*manager.getFigure(circle)->radius, 10, 1e-6);
        manager.updateRequirementParam(diameter, 30);
        EXPECT_FALSE(solve(manager, circle));
        EXPECT_GT(*manager.getFigure(circle)->radius, 0);
        EXPECT_TRUE(std::isfinite(*manager.getFigure(circle)->radius));

        DCMManager fixed;
        auto fixedCircle = fixed.addFigure(FigureDescriptor::circle(1, 2, 3));
        fixed.addRequirement(RequirementDescriptor::fixCircle(fixedCircle));
        fixed.addRequirement(RequirementDescriptor::circleRadius(fixedCircle, 10));
        fixed.setSolveMode(mode);
        EXPECT_FALSE(solve(fixed, fixedCircle));
        EXPECT_DOUBLE_EQ(*fixed.getFigure(fixedCircle)->radius, 3);
    }
}

TEST(DCMManagerSizeConstraints, EqualLengthsSolveWithFixedSegmentAndZeroLengthSegment) {
    for (auto mode : modes) for (bool collapsed : {false, true}) {
        DCMManager manager;
        auto first = manager.addFigure(FigureDescriptor::line(0, 0, 3, 4));
        auto second = manager.addFigure(FigureDescriptor::line(10, 10, collapsed ? 10 : 12, 10));
        manager.addRequirement(RequirementDescriptor::fixLine(first));
        manager.addRequirement(RequirementDescriptor::equalLength(first, second));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, second));
        EXPECT_NEAR(length(manager, second), 5, 1e-6);
        EXPECT_EQ(manager.getFigure(first)->coords, (std::vector<double>{0, 0, 3, 4}));
        expectFreedom(manager, 3);
        ASSERT_TRUE(solve(manager, second));
    }
}

TEST(DCMManagerSizeConstraints, EqualLengthsHaveNoAbsoluteTargetAndCombineSharedAliases) {
    for (auto mode : modes) {
        DCMManager manager;
        auto first = manager.addFigure(FigureDescriptor::line(0, 0, 3, 4));
        auto second = manager.addFigure(FigureDescriptor::line(10, 0, 12, 0));
        manager.addRequirement(RequirementDescriptor::equalLength(first, second));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, first));
        EXPECT_NEAR(length(manager, first), length(manager, second), 1e-6);
        expectFreedom(manager, 7);

        DCMManager shared;
        auto a = shared.addFigure(FigureDescriptor::point(0, 0));
        auto alias = shared.addFigure(FigureDescriptor::point(0, 0));
        auto b = shared.addFigure(FigureDescriptor::point(3, 4));
        auto c = shared.addFigure(FigureDescriptor::point(2, 0));
        auto ab = shared.addFigure(FigureDescriptor::line(a, b));
        auto ac = shared.addFigure(FigureDescriptor::line(alias, c));
        shared.addRequirement(RequirementDescriptor::pointOnPoint(a, alias));
        shared.addRequirement(RequirementDescriptor::fixLine(ab));
        shared.addRequirement(RequirementDescriptor::equalLength(ab, ac));
        shared.setSolveMode(mode);
        ASSERT_TRUE(solve(shared, ac));
        EXPECT_NEAR(length(shared, ac), 5, 1e-6);
        expectPoint(shared, a, 0, 0);
        expectPoint(shared, alias, 0, 0);
        expectPoint(shared, b, 3, 4);
        expectFreedom(shared, 1);
    }
}

TEST(DCMManagerSizeConstraints, EqualLengthsWeightsRemovalSnapshotsAndContradictions) {
    for (auto mode : modes) {
        DCMManager manager;
        auto first = manager.addFigure(FigureDescriptor::line(0, 0, 3, 4));
        auto second = manager.addFigure(FigureDescriptor::line(10, 0, 12, 0));
        manager.addRequirement(RequirementDescriptor::fixLine(first));
        auto equal = manager.addRequirement(RequirementDescriptor::equalLength(first, second));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, second));
        const auto saved = manager.snapshot();
        manager.updateRequirementWeight(equal, 0);
        manager.updateLine({second, 10, 0, 12, 0});
        ASSERT_TRUE(solve(manager, second));
        EXPECT_NEAR(length(manager, second), 2, 1e-12);
        expectFreedom(manager, 4);
        manager.updateRequirementWeight(equal, 2);
        ASSERT_TRUE(solve(manager, second));
        EXPECT_NEAR(length(manager, second), 5, 1e-6);
        manager.removeRequirement(equal);
        EXPECT_NE(manager.getComponentForFigure(first), manager.getComponentForFigure(second));
        manager.restoreSnapshot(saved);
        ASSERT_TRUE(solve(manager, second));
        expectFreedom(manager, 3);
        manager.updateRequirementWeight(equal, 0);
        manager.updateLine({second, 10, 0, 12, 0});
        manager.addRequirement(RequirementDescriptor::fixLine(second));
        manager.updateRequirementWeight(equal, 1);
        EXPECT_FALSE(solve(manager, second));
        EXPECT_NEAR(length(manager, first), 5, 1e-12);
        EXPECT_NEAR(length(manager, second), 2, 1e-12);
        manager.removeFigure(second, true);
        EXPECT_FALSE(manager.hasRequirement(equal));
    }
    DCMManager zero;
    auto first = zero.addFigure(FigureDescriptor::line(0, 0, 0, 0));
    auto second = zero.addFigure(FigureDescriptor::line(1, 1, 1, 1));
    zero.addRequirement(RequirementDescriptor::equalLength(first, second));
    EXPECT_TRUE(zero.solve());
}

TEST(DCMManagerSizeConstraints, EqualRadiiSolveTogetherOrFollowAPrescribedOrFixedRadius) {
    for (auto mode : modes) for (int sizeMode : {0, 1, 2}) {
        DCMManager manager;
        auto first = manager.addFigure(FigureDescriptor::circle(1, 2, 3));
        auto second = manager.addFigure(FigureDescriptor::circle(5, 6, 7));
        manager.addRequirement(RequirementDescriptor::equalRadius(first, second));
        if (sizeMode == 1) manager.addRequirement(RequirementDescriptor::circleRadius(first, 4));
        if (sizeMode == 2) manager.addRequirement(RequirementDescriptor::fixCircle(first));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, second));
        const auto a = manager.getFigure(first).value(), b = manager.getFigure(second).value();
        EXPECT_NEAR(*a.radius, *b.radius, 1e-6);
        if (sizeMode == 1) EXPECT_NEAR(*a.radius, 4, 1e-6);
        if (sizeMode == 2) EXPECT_DOUBLE_EQ(*a.radius, 3);
        EXPECT_EQ(a.coords, (std::vector<double>{1, 2}));
        EXPECT_EQ(b.coords, (std::vector<double>{5, 6}));
        expectFreedom(manager, sizeMode == 0 ? 5 : sizeMode == 1 ? 4 : 2);
        const auto& row = manager.getRequirementSystem().getFunctions().front();
        EXPECT_EQ(row->getVars().size(), 2u);
    }
}

TEST(DCMManagerSizeConstraints, EqualRadiiWeightsRemovalSnapshotsDragAndConflicts) {
    for (auto mode : modes) {
        DCMManager manager;
        auto first = manager.addFigure(FigureDescriptor::circle(1, 2, 3));
        auto second = manager.addFigure(FigureDescriptor::circle(5, 6, 7));
        manager.addRequirement(RequirementDescriptor::fixCircle(first));
        auto equal = manager.addRequirement(RequirementDescriptor::equalRadius(first, second));
        manager.setSolveMode(mode);
        ASSERT_TRUE(solve(manager, second));
        const auto saved = manager.snapshot();
        manager.updateRequirementWeight(equal, 0);
        manager.updateCircle({second, 8});
        ASSERT_TRUE(solve(manager, second));
        EXPECT_DOUBLE_EQ(*manager.getFigure(second)->radius, 8);
        expectFreedom(manager, 3);
        manager.updateRequirementWeight(equal, 2);
        ASSERT_TRUE(solve(manager, second));
        EXPECT_NEAR(*manager.getFigure(second)->radius, 3, 1e-6);
        if (mode == SolveMode::DRAG) {
            manager.updateCircle({second, 9});
            EXPECT_NEAR(*manager.getFigure(second)->radius, 3, 1e-6);
        }
        manager.removeRequirement(equal);
        EXPECT_NE(manager.getComponentForFigure(first), manager.getComponentForFigure(second));
        manager.restoreSnapshot(saved);
        ASSERT_TRUE(solve(manager, second));
        manager.addRequirement(RequirementDescriptor::circleDiameter(second, 12));
        EXPECT_FALSE(solve(manager, second));
        EXPECT_DOUBLE_EQ(*manager.getFigure(first)->radius, 3);
        EXPECT_GT(*manager.getFigure(second)->radius, 0);
        manager.removeFigure(second, true);
        EXPECT_FALSE(manager.hasRequirement(equal));
    }
}

TEST(DCMManagerSizeConstraints, LocalSolvesDoNotApplySizesInOtherComponents) {
    DCMManager manager;
    auto first = manager.addFigure(FigureDescriptor::circle(0, 0, 2));
    auto second = manager.addFigure(FigureDescriptor::circle(5, 5, 4));
    auto other = manager.addFigure(FigureDescriptor::circle(100, 100, 1));
    manager.addRequirement(RequirementDescriptor::circleRadius(first, 3));
    auto equal = manager.addRequirement(RequirementDescriptor::equalRadius(first, second));
    manager.addRequirement(RequirementDescriptor::circleDiameter(other, 20));
    manager.setSolveMode(SolveMode::LOCAL);
    ASSERT_TRUE(solve(manager, second));
    EXPECT_NEAR(*manager.getFigure(first)->radius, 3, 1e-6);
    EXPECT_NEAR(*manager.getFigure(second)->radius, 3, 1e-6);
    EXPECT_DOUBLE_EQ(*manager.getFigure(other)->radius, 1);
    manager.removeRequirement(equal);
    EXPECT_NE(manager.getComponentForFigure(first), manager.getComponentForFigure(second));
}

TEST(DCMManagerSizeConstraints, DescriptorsValidateSizesArityTypesAndAtomicParameterEdits) {
    // Existing serialized type values remain stable.
    static_assert(static_cast<int>(RequirementType::ET_FIXCIRCLE) == 15);
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(RequirementDescriptor::circleRadius(ID(1), invalid).validate(), std::invalid_argument);
        EXPECT_THROW(RequirementDescriptor::circleDiameter(ID(1), invalid).validate(), std::invalid_argument);
    }
    EXPECT_THROW(RequirementDescriptor::circleDiameter(ID(1), std::numeric_limits<double>::denorm_min()).validate(), std::invalid_argument);
    EXPECT_NO_THROW(RequirementDescriptor::circleDiameter(ID(1), std::numeric_limits<double>::max()).validate());
    for (auto type : {RequirementType::ET_CIRCLERADIUS, RequirementType::ET_CIRCLEDIAMETER}) {
        EXPECT_THROW(RequirementDescriptor(type, {ID(1)}).validate(), std::invalid_argument);
        EXPECT_THROW(RequirementDescriptor(type, {ID(1), ID(2)}, 3).validate(), std::invalid_argument);
    }
    for (auto type : {RequirementType::ET_POINTONCIRCLE, RequirementType::ET_EQUALLENGTH, RequirementType::ET_EQUALRADIUS}) {
        EXPECT_THROW(RequirementDescriptor(type, {ID(1)}).validate(), std::invalid_argument);
        EXPECT_THROW(RequirementDescriptor(type, {ID(1), ID(2)}, 3).validate(), std::invalid_argument);
    }
    DCMManager manager;
    auto point = manager.addFigure(FigureDescriptor::point(0, 0));
    auto line = manager.addFigure(FigureDescriptor::line(1, 1, 2, 2));
    auto circle = manager.addFigure(FigureDescriptor::circle(3, 3, 2));
    auto arc = manager.addFigure(FigureDescriptor::arc(0, 1, 2, 1, 1, 0));
    for (auto bad : {RequirementDescriptor::pointOnCircle(circle, point), RequirementDescriptor::circleRadius(line, 2),
                     RequirementDescriptor::circleDiameter(arc, 4), RequirementDescriptor::equalLength(line, circle),
                     RequirementDescriptor::equalRadius(circle, arc), RequirementDescriptor::equalRadius(circle, ID(9999))}) {
        EXPECT_THROW(manager.addRequirement(bad), std::runtime_error);
        EXPECT_EQ(manager.requirementCount(), 0u);
    }
    auto size = manager.addRequirement(RequirementDescriptor::circleDiameter(circle, 6));
    ASSERT_TRUE(manager.solve());
    EXPECT_THROW(manager.updateRequirementParam(size, 0), std::invalid_argument);
    EXPECT_DOUBLE_EQ(*manager.getRequirement(size)->param, 6);
    EXPECT_NEAR(*manager.getFigure(circle)->radius, 3, 1e-6);
    auto equal = manager.addRequirement(RequirementDescriptor::equalRadius(circle, circle));
    EXPECT_THROW(manager.updateRequirementParam(equal, 2), std::runtime_error);
}

TEST(DCMManagerSizeConstraints, FactoryUsesMathRadiusUnitsAndSharedVariables) {
    DCMManager manager;
    auto circleId = manager.addFigure(FigureDescriptor::circle(0, 0, 3));
    auto lineId = manager.addFigure(FigureDescriptor::line(0, 0, 3, 4));
    auto pointId = manager.addFigure(FigureDescriptor::point(3, 4));
    auto* circle = manager.storage().get<Figures::Circle2D>(circleId);
    auto* line = manager.storage().get<Figures::Line2D>(lineId);
    auto* point = manager.storage().get<Figures::Point2D>(pointId);
    using Factory = Function::RequirementFunctionFactory;
    auto radius = Factory::createCircleRadius(circle, 5);
    auto diameter = Factory::createCircleDiameter(circle, 10);
    EXPECT_DOUBLE_EQ(radius->mathematical()->evaluate(), diameter->mathematical()->evaluate());
    EXPECT_EQ(radius->mathematical()->gradient(), diameter->mathematical()->gradient());
    EXPECT_DOUBLE_EQ(Factory::createPointOnCircle(point, circle)->mathematical()->evaluate(), 2);
    EXPECT_DOUBLE_EQ(Factory::createEqualLength(line, line)->mathematical()->evaluate(), 0);
    EXPECT_DOUBLE_EQ(Factory::createEqualRadius(circle, circle)->mathematical()->evaluate(), 0);
    EXPECT_THROW(Factory::createCircleDiameter(circle, 0), std::invalid_argument);
}
