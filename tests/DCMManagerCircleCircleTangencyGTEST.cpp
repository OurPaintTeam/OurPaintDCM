#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;
namespace {
constexpr SolveMode modes[]={SolveMode::GLOBAL,SolveMode::LOCAL,SolveMode::DRAG};
constexpr CircleTangencyKind kinds[]={CircleTangencyKind::EXTERNAL,
    CircleTangencyKind::FIRST_CONTAINS_SECOND,CircleTangencyKind::SECOND_CONTAINS_FIRST};
double distance(const DCMManager& m,ID a,ID b) {
    auto x=m.getFigure(a)->coords,y=m.getFigure(b)->coords;
    return std::hypot(y[0]-x[0],y[1]-x[1]);
}
}

TEST(DCMManagerCircleCircleTangency, AllBranchesSolveAndPersistAcrossEditsAndSnapshots) {
    for (auto mode:modes) for (auto kind:kinds) {
        DCMManager m;
        double r1=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST ? 2 : 5;
        double r2=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST ? 5 : 2;
        auto c1=m.addFigure(FigureDescriptor::circle(0,0,r1));
        auto c2=m.addFigure(FigureDescriptor::circle(9,4,r2));
        m.addRequirement(RequirementDescriptor::fixCircle(c1));
        m.addRequirement(RequirementDescriptor::circleRadius(c2,r2));
        auto id=m.addRequirement(RequirementDescriptor::circleCircleTangent(c1,c2,kind));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(c2)));
        EXPECT_NEAR(distance(m,c1,c2),kind==CircleTangencyKind::EXTERNAL ? r1+r2 : std::abs(r1-r2),1e-6);
        EXPECT_EQ(m.getFigure(c1)->coords,(std::vector<double>{0,0}));
        EXPECT_EQ(m.getFigure(c1)->radius,r1);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,1u);
        auto saved=m.snapshot();
        auto other=kind==CircleTangencyKind::EXTERNAL ? CircleTangencyKind::FIRST_CONTAINS_SECOND : CircleTangencyKind::EXTERNAL;
        m.updateCircleCircleTangencyKind(id,other); ASSERT_TRUE(m.solve(m.getComponentForFigure(c2)));
        EXPECT_NEAR(distance(m,c1,c2),other==CircleTangencyKind::EXTERNAL ? r1+r2 : std::abs(r1-r2),1e-6);
        m.restoreSnapshot(saved); EXPECT_EQ(m.getRequirement(id)->param,static_cast<double>(kind));
        m.updateRequirementWeight(id,0); m.updateCircle({c2,9,4,r2});
        ASSERT_TRUE(m.solve(m.getComponentForFigure(c2))); EXPECT_NEAR(distance(m,c1,c2),std::hypot(9,4),1e-6);
        m.updateRequirementWeight(id,2); ASSERT_TRUE(m.solve(m.getComponentForFigure(c2)));
        if (mode==SolveMode::DRAG) {
            m.updateCircle({c2,8,6,r2});
            EXPECT_NEAR(distance(m,c1,c2),kind==CircleTangencyKind::EXTERNAL ? r1+r2 : std::abs(r1-r2),1e-6);
        }
        m.removeRequirement(id); EXPECT_NE(m.getComponentForFigure(c1),m.getComponentForFigure(c2));
        m.restoreSnapshot(saved); m.removeFigure(c1,true); EXPECT_FALSE(m.hasRequirement(id));
    }
}

TEST(DCMManagerCircleCircleTangency, CoincidentExternalCentersSeparateAndFreeCirclesHaveFiveDoF) {
    for (auto mode:modes) {
        DCMManager m;
        auto c1=m.addFigure(FigureDescriptor::circle(0,0,3));
        auto c2=m.addFigure(FigureDescriptor::circle(0,0,2));
        m.addRequirement(RequirementDescriptor::fixCircle(c1));
        m.addRequirement(RequirementDescriptor::circleRadius(c2,2));
        m.addRequirement(RequirementDescriptor::circleCircleTangent(c1,c2,CircleTangencyKind::EXTERNAL));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(c2)));
        EXPECT_NEAR(distance(m,c1,c2),5,1e-6);
        DCMManager free;
        auto a=free.addFigure(FigureDescriptor::circle(0,0,3));
        auto b=free.addFigure(FigureDescriptor::circle(4,2,2));
        free.addRequirement(RequirementDescriptor::circleCircleTangent(a,b,CircleTangencyKind::EXTERNAL));
        free.setSolveMode(mode); ASSERT_TRUE(free.solve(free.getComponentForFigure(a)));
        EXPECT_EQ(free.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,5u);
    }
}

TEST(DCMManagerCircleCircleTangency, WrongOrderAndCoincidentInternalCirclesCannotPass) {
    EXPECT_THROW(RequirementDescriptor::circleCircleTangent(ID(1),ID(2),
        static_cast<CircleTangencyKind>(3)).validate(),std::invalid_argument);
    for (auto mode:modes) for (bool same:{false,true}) {
        DCMManager m;
        auto c1=m.addFigure(FigureDescriptor::circle(0,0,2));
        auto c2=m.addFigure(FigureDescriptor::circle(same?0:1,0,same?2:3));
        auto id=m.addRequirement(RequirementDescriptor::circleCircleTangent(c1,c2,CircleTangencyKind::FIRST_CONTAINS_SECOND));
        m.setSolveMode(mode); EXPECT_FALSE(m.solve(m.getComponentForFigure(c1)));
        EXPECT_EQ(m.getRequirementSystem().diagnose(),SystemStatus::SINGULAR_SYSTEM);
        EXPECT_EQ(m.getFigure(c1)->radius,2); EXPECT_EQ(m.getFigure(c2)->radius,same?2:3);
        EXPECT_THROW(m.updateRequirementParam(id,0.5),std::invalid_argument);
        EXPECT_EQ(m.getRequirement(id)->param,1);
    }
}

TEST(DCMManagerCircleCircleTangency, SharedCenterWithFixedSizesIsIncompatibleInAllModes) {
    for (auto mode:modes) {
        DCMManager m;
        auto p=m.addFigure(FigureDescriptor::point(0,0));
        auto q=m.addFigure(FigureDescriptor::point(0,0));
        auto c1=m.addFigure(FigureDescriptor::circle(p,3));
        auto c2=m.addFigure(FigureDescriptor::circle(q,2));
        m.addRequirement(RequirementDescriptor::pointOnPoint(p,q));
        m.addRequirement(RequirementDescriptor::fixCircle(c1));
        m.addRequirement(RequirementDescriptor::circleRadius(c2,2));
        m.addRequirement(RequirementDescriptor::circleCircleTangent(c1,c2,CircleTangencyKind::EXTERNAL));
        m.setSolveMode(mode); EXPECT_FALSE(m.solve(m.getComponentForFigure(c2)));
        EXPECT_GT(*m.getFigure(c2)->radius,0);
    }
}
