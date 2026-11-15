#include "DCMManager.h"
#include <gtest/gtest.h>
#include <numbers>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerLineArcMidpoint, MovesLineToMinorAndMajorArcMidpointDuringSolveAndDrag) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double endY : {-1.0, 1.0}) {
            DCMManager m;
            const auto arc = m.addFigure(FigureDescriptor::arc(1, 0, 0, endY, 0, 0));
            const auto line = m.addFigure(FigureDescriptor::line(10, 12, 14, 16));
            m.addRequirement(RequirementDescriptor::fixArc(arc));
            const auto requirement = m.addRequirement(RequirementDescriptor::lineArcMidpoint(line, arc));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
            const double expected = endY*std::sqrt(0.5);
            auto d = m.getFigure(line)->coords;
            EXPECT_NEAR((d[0]+d[2])/2, expected, 1e-6);
            EXPECT_NEAR((d[1]+d[3])/2, std::sqrt(0.5), 1e-6);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);
            if (mode == SolveMode::DRAG) {
                ASSERT_TRUE(m.updateLine({line, 10, 10, 16, 18}));
                d = m.getFigure(line)->coords;
                EXPECT_NEAR((d[0]+d[2])/2, expected, 1e-6);
                EXPECT_NEAR((d[1]+d[3])/2, std::sqrt(0.5), 1e-6);
            }
            const auto saved = m.snapshot();
            m.updateRequirementWeight(requirement, 0);
            ASSERT_TRUE(m.updateLine({line, 10, 10, 16, 18}));
            ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
            EXPECT_EQ(m.getFigure(line)->coords, (std::vector<double>{10, 10, 16, 18}));
            m.restoreSnapshot(saved);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
            m.removeRequirement(requirement);
            EXPECT_NE(m.getComponentForFigure(line), m.getComponentForFigure(arc));
        }
    }
}

TEST(DCMManagerLineArcMidpoint, FixedLineMovesArcInEveryModeAndAfterDraggingEndpoints) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager m;
        const auto line = m.addFigure(FigureDescriptor::line(1, 2, 3, 2));
        const auto arc = m.addFigure(FigureDescriptor::arc(1, 0, 0, 1, 0, 0));
        m.addRequirement(RequirementDescriptor::fixLine(line));
        m.addRequirement(RequirementDescriptor::fixPoint(m.getFigure(arc)->pointIds[2]));
        m.addRequirement(RequirementDescriptor::lineArcMidpoint(line, arc));
        m.setSolveMode(mode);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
        if (mode == SolveMode::DRAG) {
            const auto d = m.getFigure(arc)->coords;
            ASSERT_TRUE(m.updateArc({arc, d[0]/2, d[1]/2, d[2]/2, d[3]/2, 0, 0}));
        }
        const auto d = m.getFigure(arc)->coords;
        const double radius = std::hypot(d[0], d[1]);
        const double a = std::atan2(d[1], d[0]);
        double b = std::atan2(d[3], d[2]);
        if (b < a) b += 2*std::numbers::pi;
        EXPECT_NEAR(radius*std::cos((a+b)/2), 2, 1e-6);
        EXPECT_NEAR(radius*std::sin((a+b)/2), 2, 1e-6);
        EXPECT_NEAR(radius, std::sqrt(8.0), 1e-6);
        EXPECT_NEAR(std::hypot(d[2], d[3]), std::sqrt(8.0), 1e-6);
        EXPECT_EQ(m.getFigure(line)->coords, (std::vector<double>{1, 2, 3, 2}));
    }
}

TEST(DCMManagerLineArcMidpoint, AliasedEndpointHasCorrectDerivativeAndNoFreeCoordinatesRemain) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(1, 0, 0, 1, 0, 0));
    const auto alias = m.addFigure(FigureDescriptor::point(1, 0));
    const auto point = m.addFigure(FigureDescriptor::point(2, 3));
    const auto line = m.addFigure(FigureDescriptor::line(alias, point));
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    m.addRequirement(RequirementDescriptor::pointOnPoint(alias, m.getFigure(arc)->pointIds[0]));
    m.addRequirement(RequirementDescriptor::lineArcMidpoint(line, arc));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(*m.getFigure(point)->x, std::sqrt(2.0)-1, 1e-6);
    EXPECT_NEAR(*m.getFigure(point)->y, std::sqrt(2.0), 1e-6);
    EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
}

TEST(DCMManagerLineArcMidpoint, CollapsedLineIsAllowedButIncompatibleFixesAndWrongTypesFail) {
    for (double y : {1.0, 2.0}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(1, 0, -1, 0, 0, 0));
        const auto line = m.addFigure(FigureDescriptor::line(0, y, 0, y));
        m.addRequirement(RequirementDescriptor::fixArc(arc));
        m.addRequirement(RequirementDescriptor::fixLine(line));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::lineArcMidpoint(arc, line)), std::runtime_error);
        auto descriptor = RequirementDescriptor::lineArcMidpoint(line, arc);
        m.addRequirement(descriptor);
        EXPECT_EQ(m.solve(), y == 1);
        descriptor.param = 1;
        EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    }
}
