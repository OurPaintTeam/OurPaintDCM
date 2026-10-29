#include "DCMManager.h"
#include <gtest/gtest.h>
#include <numbers>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
std::pair<double, double> midpoint(const DCMManager& m, ID arc) {
    const auto d = m.getFigure(arc)->coords;
    const double a = std::atan2(d[1]-d[5], d[0]-d[4]);
    double b = std::atan2(d[3]-d[5], d[2]-d[4]);
    if (b < a) b += 2*std::numbers::pi;
    const double radius = std::hypot(d[0]-d[4], d[1]-d[5]);
    return {d[4]+radius*std::cos((a+b)/2), d[5]+radius*std::sin((a+b)/2)};
}
void expectMidpoints(const DCMManager& m, ID first, ID second) {
    const auto a = midpoint(m, first), b = midpoint(m, second);
    EXPECT_NEAR(a.first, b.first, 1e-6);
    EXPECT_NEAR(a.second, b.second, 1e-6);
}
}

TEST(DCMManagerArcArcMidpoint, SolvesMajorAndMinorArcsAndDragInEveryMode) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (double endY : {-1.0, 1.0}) {
            DCMManager m;
            const auto first = m.addFigure(FigureDescriptor::arc(1, 0, 0, endY, 0, 0));
            const auto second = m.addFigure(FigureDescriptor::arc(5, 0, 4, 1, 4, 0));
            const auto points = m.getFigure(second)->pointIds;
            m.addRequirement(RequirementDescriptor::fixArc(first));
            m.addRequirement(RequirementDescriptor::pointPointDist(points[0], points[2], 1));
            m.addRequirement(RequirementDescriptor::arcArcMidpoint(first, second));
            m.setSolveMode(mode);
            ASSERT_TRUE(m.solve(m.getComponentForFigure(second)));
            ASSERT_TRUE(m.solve(m.getComponentForFigure(second)));
            expectMidpoints(m, first, second);
            EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);
            if (mode == SolveMode::DRAG) {
                const auto d = m.getFigure(second)->coords;
                ASSERT_TRUE(m.updateArc({second, d[0]+10, d[1]+5, d[2]+10, d[3]+5, d[4]+10, d[5]+5}));
                expectMidpoints(m, first, second);
            }
            EXPECT_EQ(m.getFigure(first)->coords, (std::vector<double>{1, 0, 0, endY, 0, 0}));
        }
    }
}

TEST(DCMManagerArcArcMidpoint, WeightsSnapshotsAndRemovalPreserveComponents) {
    DCMManager m;
    const auto first = m.addFigure(FigureDescriptor::arc(1, 0, 0, 1, 0, 0));
    const auto second = m.addFigure(FigureDescriptor::arc(3, 2, 2, 3, 2, 2));
    m.addRequirement(RequirementDescriptor::fixArc(first));
    const auto requirement = m.addRequirement(RequirementDescriptor::arcArcMidpoint(first, second));
    ASSERT_TRUE(m.solve());
    const auto saved = m.snapshot();
    m.updateRequirementWeight(requirement, 0);
    ASSERT_TRUE(m.updateArc({second, 3, 2, 2, 3, 2, 2}));
    ASSERT_TRUE(m.solve());
    EXPECT_EQ(m.getFigure(second)->coords, (std::vector<double>{3, 2, 2, 3, 2, 2}));
    m.restoreSnapshot(saved);
    ASSERT_TRUE(m.solve());
    expectMidpoints(m, first, second);
    m.removeRequirement(requirement);
    EXPECT_NE(m.getComponentForFigure(first), m.getComponentForFigure(second));
}

TEST(DCMManagerArcArcMidpoint, RejectsWrongTypesParametersAndIncompatibleFixedArcs) {
    for (bool collapsed : {false, true}) {
        DCMManager m;
        const auto first = m.addFigure(FigureDescriptor::arc(1, 0, 0, 1, 0, 0));
        const auto second = m.addFigure(FigureDescriptor::arc(3, 2, collapsed ? 3 : 2, collapsed ? 2 : 3, 2, 2));
        const auto point = m.addFigure(FigureDescriptor::point(0, 0));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcArcMidpoint(first, point)), std::runtime_error);
        m.addRequirement(RequirementDescriptor::fixArc(first));
        m.addRequirement(RequirementDescriptor::fixArc(second));
        auto descriptor = RequirementDescriptor::arcArcMidpoint(first, second);
        m.addRequirement(descriptor);
        EXPECT_FALSE(m.solve());
        descriptor.param = 1;
        EXPECT_THROW(descriptor.validate(), std::invalid_argument);
        descriptor.param.reset(); descriptor.objectIds.pop_back();
        EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    }
}
