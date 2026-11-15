#include "DCMManager.h"
#include <gtest/gtest.h>
#include <numbers>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerArcMidpoint, FindsHalfLengthCounterclockwiseInEveryModeAndDuringDrag) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double sweep : {20.0, 90.0, 180.0, 270.0, 359.0}) {
            DCMManager m;
            const double start = 350 * std::numbers::pi / 180;
            const double end = start + sweep * std::numbers::pi / 180;
            const auto arc = m.addFigure(FigureDescriptor::arc(5 * std::cos(start), 5 * std::sin(start),
                5 * std::cos(end), 5 * std::sin(end), 0, 0));
            const auto point = m.addFigure(FigureDescriptor::point(-3, 8));
            m.addRequirement(RequirementDescriptor::fixArc(arc));
            m.addRequirement(RequirementDescriptor::pointAtArcMidpoint(point, arc));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(point)));
            EXPECT_NEAR(*m.getFigure(point)->x, 5 * std::cos((start + end) / 2), 1e-6);
            EXPECT_NEAR(*m.getFigure(point)->y, 5 * std::sin((start + end) / 2), 1e-6);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updatePoint({point, 20, 20}));
                EXPECT_NEAR(*m.getFigure(point)->x, 5 * std::cos((start + end) / 2), 1e-6);
                EXPECT_NEAR(*m.getFigure(point)->y, 5 * std::sin((start + end) / 2), 1e-6);
            }
        }
    }
}

TEST(DCMManagerArcMidpoint, FixedMidpointMovesArcAndWeightsSnapshotsRemovalWork) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(1, 0, 0, 1, 0, 0));
    const auto point = m.addFigure(FigureDescriptor::point(2, 2));
    const auto center = m.getFigure(arc)->pointIds[2];
    m.addRequirement(RequirementDescriptor::fixPoint(point));
    m.addRequirement(RequirementDescriptor::fixPoint(center));
    const auto midpoint = m.addRequirement(RequirementDescriptor::pointAtArcMidpoint(point, arc));
    ASSERT_TRUE(m.solve());
    const auto d = m.getFigure(arc)->coords;
    EXPECT_NEAR(std::hypot(d[0], d[1]), std::hypot(d[2], d[3]), 1e-6);
    const auto saved = m.snapshot();
    m.updateRequirementWeight(midpoint, 0);
    ASSERT_TRUE(m.updateArc({arc, 1, 0, 0, 1, 0, 0}));
    ASSERT_TRUE(m.solve());
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    m.removeRequirement(midpoint);
    EXPECT_NE(m.getComponentForFigure(point), m.getComponentForFigure(arc));
}

TEST(DCMManagerArcMidpoint, WrongTypesDegenerateArcsAndFixedWrongMidpointsFail) {
    for (bool collapsed : {false, true}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(1, 0, collapsed ? 1 : -1, 0, 0, 0));
        const auto point = m.addFigure(FigureDescriptor::point(0, -1));
        m.addRequirement(RequirementDescriptor::fixArc(arc));
        m.addRequirement(RequirementDescriptor::fixPoint(point));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointAtArcMidpoint(arc, point)), std::runtime_error);
        m.addRequirement(RequirementDescriptor::pointAtArcMidpoint(point, arc));
        EXPECT_FALSE(m.solve());
    }
}
