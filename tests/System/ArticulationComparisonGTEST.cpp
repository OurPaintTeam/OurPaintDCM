#include <gtest/gtest.h>
#include "../support/ArticulationComparison.h"

using namespace ArticulationComparison;

TEST(ArticulationComparisonTest, SplitAndJointSolutionsAgreeForDeterminedSystems) {
    for (const auto* shape : {"chain", "star", "cycle"}) {
        SCOPED_TRACE(shape);
        Model split(12, shape), joint(12, shape);
        JointSolve baseline(joint);
        for (double perturbation : {0.0, 0.3, -0.5}) {
            split.perturb(perturbation);
            joint.perturb(perturbation);
            ASSERT_TRUE(split.solve());
            ASSERT_TRUE(baseline.solve());
            EXPECT_TRUE(split.satisfied());
            for (std::size_t i = 0; i < split.points.size(); ++i) {
                const auto a = split.manager.getFigure(split.points[i]).value();
                const auto b = joint.manager.getFigure(joint.points[i]).value();
                EXPECT_NEAR(*a.x, *b.x, 1e-6);
                EXPECT_NEAR(*a.y, *b.y, 1e-6);
            }
        }
    }
}

TEST(ArticulationComparisonTest, UnderdeterminedSystemsSatisfySameOriginalConstraints) {
    DCMManager split, joint;
    const auto a = split.addFigure(FigureDescriptor::point(0, 0));
    const auto b = split.addFigure(FigureDescriptor::point(6, 1));
    const auto c = split.addFigure(FigureDescriptor::point(11, -2));
    split.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
    split.addRequirement(RequirementDescriptor::pointPointDist(b, c, 5));
    split.setSolveMode(SolveMode::LOCAL);
    joint.restoreSnapshot(split.snapshot());
    JointSolve baseline(joint);
    ASSERT_TRUE(split.solve(split.getComponentForFigure(a)));
    ASSERT_TRUE(baseline.solve());
    EXPECT_TRUE(satisfied(split));
    EXPECT_TRUE(satisfied(joint));
}

TEST(ArticulationComparisonTest, FallbackAndContradictionsMatchOrdinarySolutionBehavior) {
    for (bool feasible : {true, false}) {
        SCOPED_TRACE(feasible);
        DCMManager split, joint;
        const auto ids = intersection(split, feasible);
        joint.restoreSnapshot(split.snapshot());
        JointSolve baseline(joint);
        EXPECT_EQ(split.solve(split.getComponentForFigure(ids[0])), feasible);
        EXPECT_EQ(baseline.solve(), feasible);
        for (auto id : ids) {
            const auto a = split.getFigure(id).value();
            const auto b = joint.getFigure(id).value();
            EXPECT_TRUE(std::isfinite(*a.x) && std::isfinite(*a.y));
            EXPECT_NEAR(*a.x, *b.x, 1e-6);
            EXPECT_NEAR(*a.y, *b.y, 1e-6);
        }
    }
}
