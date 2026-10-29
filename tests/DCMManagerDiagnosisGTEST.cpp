#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
void expectDiagnosis(const DCMManager& manager, SystemStatus status, std::size_t freedom) {
    const auto& system = manager.getRequirementSystem();
    const auto report = system.diagnoseDetailed();
    EXPECT_EQ(report.status, status);
    EXPECT_EQ(system.diagnose(), status);
    ASSERT_TRUE(report.rank.has_value());
    EXPECT_EQ(report.degreesOfFreedom, freedom);
    EXPECT_EQ(report.variableCount - *report.rank, freedom);
    // The whole-sketch contract also applies through the base-class interface.
    const System::RequirementFunctionSystem& base = system;
    EXPECT_EQ(base.diagnose(), status);
}
}

TEST(DCMManagerDiagnosis, FixedAndFreePointsHaveTwoDegreesOfFreedomInEverySolveMode) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager manager;
        manager.setSolveMode(mode);
        auto fixed = manager.addFigure(FigureDescriptor::point(1, 2));
        manager.addFigure(FigureDescriptor::point(10, 20));
        manager.addRequirement(RequirementDescriptor::fixPoint(fixed));
        expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
        const auto report = manager.getRequirementSystem().diagnoseDetailed();
        EXPECT_EQ(report.variableCount, 4u);
        EXPECT_EQ(report.constraintCount, 2u);
        EXPECT_EQ(report.rank, 2u);
    }
}

TEST(DCMManagerDiagnosis, EmptySketchAndUnconstrainedGeometryAreDifferent) {
    DCMManager manager;
    expectDiagnosis(manager, SystemStatus::EMPTY, 0);
    auto point = manager.addFigure(FigureDescriptor::point(1, 2));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    auto line = manager.addFigure(FigureDescriptor::line(0, 0, 4, 0));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 6);
    auto circle = manager.addFigure(FigureDescriptor::circle(3, 3, 2));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 9);
    manager.removeFigure(circle, true);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 6);
    manager.removeFigure(line, true);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    manager.removeFigure(point);
    expectDiagnosis(manager, SystemStatus::EMPTY, 0);
}

TEST(DCMManagerDiagnosis, AddingAndRemovingGeometryAndFixesUpdatesCachedScene) {
    DCMManager manager;
    auto a = manager.addFigure(FigureDescriptor::point(1, 2));
    auto fixA = manager.addRequirement(RequirementDescriptor::fixPoint(a));
    expectDiagnosis(manager, SystemStatus::WELL_CONSTRAINED, 0);
    auto b = manager.addFigure(FigureDescriptor::point(3, 4));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    auto fixB = manager.addRequirement(RequirementDescriptor::fixPoint(b));
    expectDiagnosis(manager, SystemStatus::WELL_CONSTRAINED, 0);
    manager.removeRequirement(fixA);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    manager.removeRequirement(fixB);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 4);
    manager.removeFigure(a);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    // Reuse of a storage slot must not reuse a cached diagnostic column.
    auto c = manager.addFigure(FigureDescriptor::point(5, 6));
    manager.addRequirement(RequirementDescriptor::fixPoint(c));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    manager.removeFigure(c, true);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
}

TEST(DCMManagerDiagnosis, CoincidenceChainsShareDegreesOfFreedomAndFollowRemoval) {
    DCMManager manager;
    auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    auto b = manager.addFigure(FigureDescriptor::point(2, 0));
    auto c = manager.addFigure(FigureDescriptor::point(4, 0));
    auto ab = manager.addRequirement(RequirementDescriptor::pointOnPoint(a, b));
    auto bc = manager.addRequirement(RequirementDescriptor::pointOnPoint(b, c));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    EXPECT_EQ(manager.getRequirementSystem().diagnoseDetailed().variableCount, 2u);
    auto fixed = manager.addRequirement(RequirementDescriptor::fixPoint(a));
    expectDiagnosis(manager, SystemStatus::WELL_CONSTRAINED, 0);
    manager.removeRequirement(bc);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    manager.removeRequirement(ab);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 4);
    manager.removeRequirement(fixed);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 6);
}

TEST(DCMManagerDiagnosis, DisabledFixesAndCoincidencesLeaveGeometryFree) {
    DCMManager manager;
    auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    auto b = manager.addFigure(FigureDescriptor::point(2, 0));
    auto fixed = manager.addRequirement(RequirementDescriptor::fixPoint(a));
    auto coincident = manager.addRequirement(RequirementDescriptor::pointOnPoint(a, b));
    expectDiagnosis(manager, SystemStatus::WELL_CONSTRAINED, 0);
    manager.updateRequirementWeight(fixed, 0);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    manager.updateRequirementWeight(coincident, 0);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 4);
    manager.updateRequirementWeight(fixed, 1);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    manager.updateRequirementWeight(coincident, 1);
    expectDiagnosis(manager, SystemStatus::WELL_CONSTRAINED, 0);
}

TEST(DCMManagerDiagnosis, SharedFigurePointsAreCountedOnceAndRadiusIsIndependent) {
    DCMManager manager;
    auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    auto b = manager.addFigure(FigureDescriptor::point(4, 0));
    auto c = manager.addFigure(FigureDescriptor::point(4, 3));
    auto line = manager.addFigure(FigureDescriptor::line(a, b));
    manager.addFigure(FigureDescriptor::line(b, c));
    auto circle = manager.addFigure(FigureDescriptor::circle(a, 2));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 7);
    manager.addRequirement(RequirementDescriptor::fixLine(line));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 3);
    manager.removeFigure(circle);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);

    DCMManager circleOnly;
    auto isolated = circleOnly.addFigure(FigureDescriptor::circle(1, 2, 3));
    circleOnly.addRequirement(RequirementDescriptor::fixCircle(isolated));
    expectDiagnosis(circleOnly, SystemStatus::WELL_CONSTRAINED, 0);
}

TEST(DCMManagerDiagnosis, NonzeroDistanceWeightsDoNotChangeDegreesOfFreedom) {
    DCMManager manager;
    auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    auto b = manager.addFigure(FigureDescriptor::point(3, 4));
    auto distance = manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    for (double weight : {1.0, 1e-12, 1e12}) {
        manager.updateRequirementWeight(distance, weight);
        expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 3);
    }
    manager.updateRequirementWeight(distance, 0);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 4);
}

TEST(DCMManagerDiagnosis, MandatoryArcConditionSurvivesExplicitRequirementChanges) {
    DCMManager manager;
    expectDiagnosis(manager, SystemStatus::EMPTY, 0);
    auto arc = manager.addFigure(FigureDescriptor::arc(-2, 0, 2, 0, 0, 1));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 5);
    EXPECT_EQ(manager.getRequirementSystem().diagnoseDetailed().constraintCount, 1u);
    auto explicitArc = manager.addRequirement(RequirementDescriptor::arcCenterOnPerpendicular(arc));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 5);
    EXPECT_EQ(manager.getRequirementSystem().diagnoseDetailed().constraintCount, 1u);
    manager.updateRequirementWeight(explicitArc, 0);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 5);
    manager.removeRequirement(explicitArc);
    const auto points = manager.getStorage().getDependencies(arc);
    manager.addRequirement(RequirementDescriptor::fixPoint(points[2]));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 3);
    manager.addRequirement(RequirementDescriptor::fixPoint(points[0]));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 1);
    manager.removeFigure(arc);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
}

TEST(DCMManagerDiagnosis, UndefinedArcDoesNotReportAValidDegreeCount) {
    DCMManager manager;
    auto arc = manager.addFigure(FigureDescriptor::arc(0, 0, 0, 0, 1, 1));
    const auto report = manager.getRequirementSystem().diagnoseDetailed();
    EXPECT_EQ(report.status, SystemStatus::SINGULAR_SYSTEM);
    EXPECT_FALSE(report.rank.has_value());
    EXPECT_FALSE(report.degreesOfFreedom.has_value());
    manager.removeFigure(arc, true);
    expectDiagnosis(manager, SystemStatus::EMPTY, 0);
}

TEST(DCMManagerDiagnosis, SnapshotRestorationUpdatesGeometryAndAliasScope) {
    DCMManager manager;
    auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    auto b = manager.addFigure(FigureDescriptor::point(2, 0));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(a, b));
    const auto saved = manager.snapshot();
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
    manager.addRequirement(RequirementDescriptor::fixPoint(a));
    manager.addFigure(FigureDescriptor::circle(10, 0, 3));
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 3);
    manager.restoreSnapshot(saved);
    expectDiagnosis(manager, SystemStatus::UNDER_CONSTRAINED, 2);
}
