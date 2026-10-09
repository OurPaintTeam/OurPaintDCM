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
