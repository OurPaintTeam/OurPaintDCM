#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerFixArc, FixesEndpointsAndCenterInEveryModeAndRestoresCapturedTargets) {
    for (auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager m;
        const auto arc = m.addFigure(FigureDescriptor::arc(0, 0, 10, 0, 5, 5));
        const auto fix = m.addRequirement(RequirementDescriptor::fixArc(arc));
        const auto coordinates = m.getFigure(arc)->coords;
        m.setSolveMode(mode);
        ASSERT_TRUE(m.updateArc({arc, 1, 2, 11, 2, 6, 7}));
        EXPECT_EQ(m.getFigure(arc)->coords, coordinates);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
        const auto saved = m.snapshot();
        m.updateRequirementWeight(fix, 0);
        ASSERT_TRUE(m.updateArc({arc, 1, 2, 11, 2, 6, 7}));
        EXPECT_NE(m.getFigure(arc)->coords, coordinates);
        m.updateRequirementWeight(fix, 1);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
        EXPECT_EQ(m.getFigure(arc)->coords, coordinates);
        m.restoreSnapshot(saved);
        EXPECT_EQ(m.getFigure(arc)->coords, coordinates);
        m.removeRequirement(fix);
        ASSERT_TRUE(m.updateArc({arc, 1, 2, 11, 2, 6, 7}));
        EXPECT_NE(m.getFigure(arc)->coords, coordinates);
    }
}

TEST(DCMManagerFixArc, ConflictingAliasedFixesFail) {
    DCMManager m;
    const auto arc = m.addFigure(FigureDescriptor::arc(0, 0, 10, 0, 5, 5));
    const auto end = m.getFigure(arc)->pointIds[0];
    const auto alias = m.addFigure(FigureDescriptor::point(20, 0));
    m.addRequirement(RequirementDescriptor::fixPoint(alias));
    m.addRequirement(RequirementDescriptor::fixArc(arc));
    m.addRequirement(RequirementDescriptor::pointOnPoint(end, alias));
    EXPECT_FALSE(m.solve());
}

TEST(DCMManagerFixArc, RejectsWrongTypesWeightsAndParameters) {
    DCMManager m;
    const auto point = m.addFigure(FigureDescriptor::point(0, 0));
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::fixArc(point)), std::runtime_error);
    auto descriptor = RequirementDescriptor::fixArc(point);
    descriptor.weight = 0.5;
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.weight = 1;
    descriptor.param = 1;
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
    descriptor.param.reset();
    descriptor.objectIds.clear();
    EXPECT_THROW(descriptor.validate(), std::invalid_argument);
}
