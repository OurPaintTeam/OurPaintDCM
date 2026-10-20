#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerHorizontalDistance, SolvesBothDirectionsAndKeepsVerticalFreedom) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double sign : {-1.0, 1.0}) {
            DCMManager m;
            const auto a = m.addFigure(FigureDescriptor::point(2, 3));
            const auto p = m.addFigure(FigureDescriptor::point(2 + sign, 17));
            m.addRequirement(RequirementDescriptor::fixPoint(a));
            const auto distance = m.addRequirement(RequirementDescriptor::pointPointHorizontalDist(a, p, 10));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
            EXPECT_NEAR(*m.getFigure(p)->x, 2 + sign * 10, 1e-6);
            EXPECT_DOUBLE_EQ(*m.getFigure(p)->y, 17);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 1u);
            m.updateRequirementParam(distance, 20);
            if (mode == SolveMode::DRAG) ASSERT_TRUE(m.updatePoint({a, 9, 9}));
            else ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
            EXPECT_NEAR(std::abs(*m.getFigure(p)->x - 2), 20, 1e-6);
            const auto saved = m.snapshot();
            m.updateRequirementWeight(distance, 0);
            ASSERT_TRUE(m.updatePoint({p, 2, 9}));
            ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
            EXPECT_DOUBLE_EQ(*m.getFigure(p)->x, 2);
            m.restoreSnapshot(saved);
            EXPECT_NEAR(std::abs(*m.getFigure(p)->x - 2), 20, 1e-6);
            m.removeRequirement(distance);
            EXPECT_NE(m.getComponentForFigure(a), m.getComponentForFigure(p));
        }
    }
}

TEST(DCMManagerHorizontalDistance, ZeroProjectionAliasesAndInvalidInput) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(2, 3));
    const auto p = m.addFigure(FigureDescriptor::point(2, 17));
    m.addRequirement(RequirementDescriptor::fixPoint(a));
    m.addRequirement(RequirementDescriptor::pointPointHorizontalDist(a, p, 10));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(std::abs(*m.getFigure(p)->x - 2), 10, 1e-6);
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointPointHorizontalDist(a, p, -1)), std::invalid_argument);
    auto missing = RequirementDescriptor::pointPointHorizontalDist(a, p, 1);
    missing.param.reset();
    EXPECT_THROW(missing.validate(), std::invalid_argument);
    m.addRequirement(RequirementDescriptor::pointOnPoint(a, p));
    EXPECT_FALSE(m.solve());
}
