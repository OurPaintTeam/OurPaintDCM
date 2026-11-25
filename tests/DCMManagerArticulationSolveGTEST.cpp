#include <gtest/gtest.h>
#include "DCMManager.h"
#include <cmath>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
double distance(const DCMManager& manager, ID a, ID b) {
    const auto first = manager.getFigure(a).value();
    const auto second = manager.getFigure(b).value();
    return std::hypot(*first.x - *second.x, *first.y - *second.y);
}

TEST(DCMManagerArticulationSolveTest, ChainSolvesInEveryModeAndWithEverySolver) {
    for (auto solver : {SolverType::LEVENBERG_MARQUARDT, SolverType::DOGLEG,
                        SolverType::GAUSS_NEWTON, SolverType::NEWTON}) {
        for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
            SCOPED_TRACE(static_cast<int>(solver));
            SCOPED_TRACE(static_cast<int>(mode));
            DCMManager manager;
            const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
            const auto b = manager.addFigure(FigureDescriptor::point(3, 0));
            const auto c = manager.addFigure(FigureDescriptor::point(8, 0));
            manager.addRequirement(RequirementDescriptor::fixPoint(a));
            manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
            manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 5));
            manager.setSolver(solver);
            manager.setSolveMode(mode);
            const auto component = manager.getComponentForFigure(a).value();
            for (int attempt = 0; attempt < 2; ++attempt) {
                ASSERT_TRUE(manager.solve(component, 1e-6));
                EXPECT_NEAR(distance(manager, a, b), 5, 1e-6);
                EXPECT_NEAR(distance(manager, b, c), 5, 1e-6);
                EXPECT_DOUBLE_EQ(*manager.getFigure(a)->x, 0);
            }
        }
    }
}

TEST(DCMManagerArticulationSolveTest, UnsuccessfulSequentialPassFallsBackToJointSolution) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager manager;
        const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
        const auto p = manager.addFigure(FigureDescriptor::point(3, 1));
        const auto b = manager.addFigure(FigureDescriptor::point(10, 0));
        manager.addRequirement(RequirementDescriptor::fixPoint(a));
        manager.addRequirement(RequirementDescriptor::fixPoint(b));
        manager.addRequirement(RequirementDescriptor::pointPointDist(a, p, 5));
        manager.addRequirement(RequirementDescriptor::pointPointDist(p, b, 7));
        manager.setSolveMode(mode);
        ASSERT_TRUE(manager.solve(manager.getComponentForFigure(p), 1e-6));
        EXPECT_NEAR(distance(manager, a, p), 5, 1e-6);
        EXPECT_NEAR(distance(manager, p, b), 7, 1e-6);
        EXPECT_NEAR(*manager.getFigure(p)->x, 3.8, 1e-6);
        EXPECT_DOUBLE_EQ(*manager.getFigure(a)->x, 0);
        EXPECT_DOUBLE_EQ(*manager.getFigure(b)->x, 10);
    }
}

TEST(DCMManagerArticulationSolveTest, GlobalSolveCombinesSplitAndUnsplitComponents) {
    DCMManager manager;
    for (const double offset : {0.0, 100.0}) {
        const auto a = manager.addFigure(FigureDescriptor::point(offset, 0));
        const auto b = manager.addFigure(FigureDescriptor::point(offset + 3, 0));
        const auto c = manager.addFigure(FigureDescriptor::point(offset + 8, 0));
        manager.addRequirement(RequirementDescriptor::fixPoint(a));
        manager.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
        manager.addRequirement(RequirementDescriptor::pointPointDist(b, c, 5));
    }
    // Mix a component without a useful split into the global solve.
    const auto circle = manager.addFigure(FigureDescriptor::circle(200, 0, 2));
    manager.addRequirement(RequirementDescriptor::circleRadius(circle, 7));
    ASSERT_TRUE(manager.solve());
    EXPECT_NEAR(*manager.getFigure(circle)->radius, 7, 1e-6);
    for (const auto& function : manager.getRequirementSystem().getFunctions())
        EXPECT_TRUE(function->mathematical()->satisfied(1e-6));
}
} // namespace
