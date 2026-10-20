#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerPointOnArc, SolvesFromCenterAndDragWithoutChangingFixedArc) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto point = m.addFigure(FigureDescriptor::point(0, 0));
        m.addRequirement(RequirementDescriptor::fixArc(arc));
        const auto requirement = m.addRequirement(RequirementDescriptor::pointOnArc(point, arc));
        m.setSolveMode(mode);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(point)));
        auto p = m.getFigure(point);
        EXPECT_NEAR(std::hypot(*p->x, *p->y), 5, 1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 1u);
        if (mode == SolveMode::DRAG) {
            ASSERT_TRUE(m.updatePoint({point, 20, 20}));
            p = m.getFigure(point);
            EXPECT_NEAR(std::hypot(*p->x, *p->y), 5, 1e-6);
        }
        EXPECT_EQ(m.getFigure(arc)->coords, (std::vector<double>{3, 4, -3, 4, 0, 0}));
        const auto saved = m.snapshot();
        m.updateRequirementWeight(requirement, 0);
        ASSERT_TRUE(m.updatePoint({point, 0, -8}));
        ASSERT_TRUE(m.solve(m.getComponentForFigure(point)));
        EXPECT_DOUBLE_EQ(*m.getFigure(point)->y, -8);
        m.restoreSnapshot(saved);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(point)));
        m.removeRequirement(requirement);
        EXPECT_NE(m.getComponentForFigure(point), m.getComponentForFigure(arc));
    }
}

TEST(DCMManagerPointOnArc, IncludesSupportingCircleOutsideAngularSectorAndRejectsWrongTypes) {
    for (double y : {-5.0, -6.0}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(3, 4, -3, 4, 0, 0));
        const auto point = m.addFigure(FigureDescriptor::point(0, y));
        m.addRequirement(RequirementDescriptor::fixArc(arc));
        m.addRequirement(RequirementDescriptor::fixPoint(point));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointOnArc(arc, point)), std::runtime_error);
        m.addRequirement(RequirementDescriptor::pointOnArc(point, arc));
        EXPECT_EQ(m.solve(), y == -5);
    }
}
