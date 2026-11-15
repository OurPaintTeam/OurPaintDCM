#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerLineMidpoint, SolvesCollapsedAndOrdinaryLinesAndSupportsDragAndSnapshots) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (bool collapsed : {false, true}) {
            DCMManager m;
            const auto first = m.addFigure(FigureDescriptor::line(1, 2, collapsed ? 1 : 7, collapsed ? 2 : 8));
            const auto second = m.addFigure(FigureDescriptor::line(10, 11, 14, 15));
            m.addRequirement(RequirementDescriptor::fixLine(first));
            const auto midpoint = m.addRequirement(RequirementDescriptor::lineLineMidpoint(first, second));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(second)));
            auto d = m.getFigure(second)->coords;
            EXPECT_NEAR((d[0] + d[2]) / 2, collapsed ? 1 : 4, 1e-6);
            EXPECT_NEAR((d[1] + d[3]) / 2, collapsed ? 2 : 5, 1e-6);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updateLine({second, 10, 10, 16, 18}));
                d = m.getFigure(second)->coords;
                EXPECT_NEAR((d[0] + d[2]) / 2, collapsed ? 1 : 4, 1e-6);
                EXPECT_NEAR((d[1] + d[3]) / 2, collapsed ? 2 : 5, 1e-6);
            }
            const auto saved = m.snapshot();
            m.updateRequirementWeight(midpoint, 0);
            ASSERT_TRUE(m.updateLine({second, 10, 10, 16, 18}));
            ASSERT_TRUE(m.solve(m.getComponentForFigure(second)));
            EXPECT_EQ(m.getFigure(second)->coords, (std::vector<double>{10, 10, 16, 18}));
            m.restoreSnapshot(saved);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(second)));
            m.removeRequirement(midpoint);
            EXPECT_NE(m.getComponentForFigure(first), m.getComponentForFigure(second));
        }
    }
}

TEST(DCMManagerLineMidpoint, SharedEndpointCancelsItsDerivativeAndWrongTypesFail) {
    DCMManager m;
    const auto a = m.addFigure(FigureDescriptor::point(1, 2));
    const auto b = m.addFigure(FigureDescriptor::point(7, 8));
    const auto c = m.addFigure(FigureDescriptor::point(9, 10));
    const auto first = m.addFigure(FigureDescriptor::line(a, b));
    const auto second = m.addFigure(FigureDescriptor::line(a, c));
    m.addRequirement(RequirementDescriptor::fixLine(first));
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::lineLineMidpoint(a, second)), std::runtime_error);
    m.addRequirement(RequirementDescriptor::lineLineMidpoint(first, second));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(*m.getFigure(c)->x, 7, 1e-6);
    EXPECT_NEAR(*m.getFigure(c)->y, 8, 1e-6);
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
}

TEST(DCMManagerLineMidpoint, IncompatibleFixedMidpointsAndParametersFail) {
    DCMManager m;
    const auto first = m.addFigure(FigureDescriptor::line(0, 0, 2, 2));
    const auto second = m.addFigure(FigureDescriptor::line(10, 10, 12, 12));
    m.addRequirement(RequirementDescriptor::fixLine(first));
    m.addRequirement(RequirementDescriptor::fixLine(second));
    auto descriptor = RequirementDescriptor::lineLineMidpoint(first, second);
    m.addRequirement(descriptor);
    EXPECT_FALSE(m.solve());
    descriptor.param = 1;
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
}
