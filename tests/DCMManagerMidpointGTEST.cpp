#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
constexpr SolveMode modes[] = {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG};
bool solve(DCMManager& m, ID id) { return m.solve(m.getComponentForFigure(id)); }
void expectPoint(const DCMManager& m, ID id, double x, double y) {
    const auto p = m.getFigure(id).value();
    EXPECT_NEAR(*p.x, x, 1e-6);
    EXPECT_NEAR(*p.y, y, 1e-6);
}
}

TEST(DCMManagerMidpoint, FixedAndCollapsedSegmentsGiveTwoIndependentRowsAndZeroFreedom) {
    for (auto mode : modes) for (bool collapsed : {false, true}) {
        DCMManager m;
        auto line = m.addFigure(FigureDescriptor::line(1, 2, collapsed ? 1 : 7, collapsed ? 2 : 8));
        auto point = m.addFigure(FigureDescriptor::point(-4, 9));
        m.addRequirement(RequirementDescriptor::fixLine(line));
        auto id = m.addRequirement(RequirementDescriptor::pointAtMidpoint(point, line));
        m.setSolveMode(mode);
        ASSERT_TRUE(solve(m, point));
        expectPoint(m, point, collapsed ? 1 : 4, collapsed ? 2 : 5);
        EXPECT_EQ(m.getFigure(line)->coords, (collapsed ? std::vector<double>{1,2,1,2} : std::vector<double>{1,2,7,8}));
        const auto report = m.getRequirementSystem().diagnoseDetailed();
        EXPECT_EQ(report.degreesOfFreedom, 0u);
        EXPECT_EQ(report.constraintCount, 6u);
        const auto& rows = m.getRequirementSystem().getFunctions();
        EXPECT_EQ(rows[4]->requirementId(), id);
        EXPECT_EQ(rows[5]->requirementId(), id);
        // Opposite coordinate errors must not cancel.
        m.setSolveMode(SolveMode::GLOBAL);
        m.updatePoint({point, collapsed ? 2 : 5, collapsed ? 1 : 4});
        EXPECT_NE(rows[4]->mathematical()->evaluate(), 0);
        EXPECT_NE(rows[5]->mathematical()->evaluate(), 0);
        ASSERT_TRUE(solve(m, point));
    }
}

TEST(DCMManagerMidpoint, FixedMidpointMovesBothEndpointsAndAliasesSumTheirDerivatives) {
    for (auto mode : modes) {
        DCMManager m;
        auto p = m.addFigure(FigureDescriptor::point(8, 5));
        auto line = m.addFigure(FigureDescriptor::line(0, 0, 10, 4));
        m.addRequirement(RequirementDescriptor::fixPoint(p));
        m.addRequirement(RequirementDescriptor::pointAtMidpoint(p, line));
        m.setSolveMode(mode);
        ASSERT_TRUE(solve(m, line));
        auto d = m.getFigure(line).value();
        EXPECT_NEAR((d.coords[0] + d.coords[2]) / 2, 8, 1e-6);
        EXPECT_NEAR((d.coords[1] + d.coords[3]) / 2, 5, 1e-6);
        expectPoint(m, p, 8, 5);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);

        DCMManager aliased;
        auto a = aliased.addFigure(FigureDescriptor::point(1, 2));
        auto b = aliased.addFigure(FigureDescriptor::point(5, 6));
        auto same = aliased.addFigure(FigureDescriptor::point(1, 2));
        auto ab = aliased.addFigure(FigureDescriptor::line(a, b));
        aliased.addRequirement(RequirementDescriptor::pointOnPoint(a, same));
        aliased.addRequirement(RequirementDescriptor::fixPoint(a));
        aliased.addRequirement(RequirementDescriptor::pointAtMidpoint(same, ab));
        aliased.setSolveMode(mode);
        ASSERT_TRUE(solve(aliased, ab));
        expectPoint(aliased, b, 1, 2);
        EXPECT_EQ(aliased.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
    }
}

TEST(DCMManagerMidpoint, WeightsRemovalSnapshotsAndDragRebuildTheSystem) {
    for (auto mode : modes) {
        DCMManager m;
        auto line = m.addFigure(FigureDescriptor::line(0, 0, 4, 6));
        auto p = m.addFigure(FigureDescriptor::point(2, 3));
        m.addRequirement(RequirementDescriptor::fixLine(line));
        auto id = m.addRequirement(RequirementDescriptor::pointAtMidpoint(p, line));
        m.setSolveMode(mode);
        ASSERT_TRUE(solve(m, p));
        auto saved = m.snapshot();
        m.updateRequirementWeight(id, 0);
        m.updatePoint({p, 8, 9});
        ASSERT_TRUE(solve(m, p));
        expectPoint(m, p, 8, 9);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 2u);
        m.updateRequirementWeight(id, 2);
        ASSERT_TRUE(solve(m, p));
        expectPoint(m, p, 2, 3);
        m.removeRequirement(id);
        EXPECT_NE(m.getComponentForFigure(p), m.getComponentForFigure(line));
        m.restoreSnapshot(saved);
        ASSERT_TRUE(solve(m, p));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom, 0u);
        if (mode == SolveMode::DRAG) {
            m.updatePoint({p, -5, 7});
            expectPoint(m, p, 2, 3);
        }
        m.removeFigure(line, true);
        EXPECT_FALSE(m.hasRequirement(id));
    }
}

TEST(DCMManagerMidpoint, InvalidDescriptorsAndIncompatibleFixesAreRejected) {
    EXPECT_THROW(RequirementDescriptor(RequirementType::ET_POINTATMIDPOINT, {ID(1)}).validate(), std::invalid_argument);
    EXPECT_THROW(RequirementDescriptor(RequirementType::ET_POINTATMIDPOINT, {ID(1),ID(2)}, 1).validate(), std::invalid_argument);
    DCMManager m;
    auto line = m.addFigure(FigureDescriptor::line(0, 0, 4, 6));
    auto p = m.addFigure(FigureDescriptor::point(3, 4));
    EXPECT_THROW(m.addRequirement(RequirementDescriptor::pointAtMidpoint(line, p)), std::runtime_error);
    m.addRequirement(RequirementDescriptor::fixLine(line));
    m.addRequirement(RequirementDescriptor::fixPoint(p));
    m.addRequirement(RequirementDescriptor::pointAtMidpoint(p, line));
    for (auto mode : modes) {
        m.setSolveMode(mode);
        EXPECT_FALSE(solve(m, p));
        expectPoint(m, p, 3, 4);
    }
}
