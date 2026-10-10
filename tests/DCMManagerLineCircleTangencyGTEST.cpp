#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;
namespace { constexpr SolveMode modes[]={SolveMode::GLOBAL,SolveMode::LOCAL,SolveMode::DRAG}; }

TEST(DCMManagerLineCircleTangency, BothSavedSidesSolveOutsideTheSegmentInAllModes) {
    for (auto mode:modes) for (auto side:{TangencySide::POSITIVE,TangencySide::NEGATIVE}) {
        DCMManager m;
        auto line=m.addFigure(FigureDescriptor::line(0,0,2,0));
        auto center=m.addFigure(FigureDescriptor::point(20,8));
        auto circle=m.addFigure(FigureDescriptor::circle(center,3));
        m.addRequirement(RequirementDescriptor::fixLine(line));
        m.addRequirement(RequirementDescriptor::circleRadius(circle,3));
        auto id=m.addRequirement(RequirementDescriptor::lineCircleTangent(line,circle,side));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(circle)));
        auto c=m.getFigure(circle)->coords;
        EXPECT_NEAR(c[0],20,1e-6); EXPECT_NEAR(c[1],static_cast<double>(side)*3,1e-6);
        EXPECT_NEAR(*m.getFigure(circle)->radius,3,1e-6);
        EXPECT_EQ(m.getFigure(line)->coords,(std::vector<double>{0,0,2,0}));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,1u);
        auto saved=m.snapshot();
        auto opposite=side==TangencySide::POSITIVE ? TangencySide::NEGATIVE : TangencySide::POSITIVE;
        m.updateLineCircleTangencySide(id,opposite);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(circle)));
        EXPECT_NEAR(m.getFigure(circle)->coords[1],static_cast<double>(opposite)*3,1e-6);
        m.restoreSnapshot(saved); EXPECT_EQ(m.getRequirement(id)->param,static_cast<double>(side));
        m.updateRequirementWeight(id,0); m.updatePoint({center,20,9});
        ASSERT_TRUE(m.solve(m.getComponentForFigure(circle)));
        EXPECT_NEAR(m.getFigure(circle)->coords[1],9,1e-6);
        m.updateRequirementWeight(id,2); ASSERT_TRUE(m.solve(m.getComponentForFigure(circle)));
        EXPECT_NEAR(m.getFigure(circle)->coords[1],static_cast<double>(side)*3,1e-6);
        if (mode==SolveMode::DRAG) {
            m.updateCircle({circle,25,8,3});
            EXPECT_NEAR(m.getFigure(circle)->coords[1],static_cast<double>(side)*3,1e-6);
        }
        m.removeRequirement(id); EXPECT_NE(m.getComponentForFigure(line),m.getComponentForFigure(circle));
        m.restoreSnapshot(saved); m.removeFigure(circle,true); EXPECT_FALSE(m.hasRequirement(id));
    }
}

TEST(DCMManagerLineCircleTangency, FreeLineAndAliasedCenterRespectFixations) {
    for (auto mode:modes) {
        DCMManager m;
        auto circle=m.addFigure(FigureDescriptor::circle(0,3,3));
        auto line=m.addFigure(FigureDescriptor::line(-4,-2,4,-1));
        m.addRequirement(RequirementDescriptor::fixCircle(circle));
        m.addRequirement(RequirementDescriptor::lineCircleTangent(line,circle,TangencySide::POSITIVE));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        EXPECT_EQ(m.getFigure(circle)->coords,(std::vector<double>{0,3}));
        EXPECT_EQ(m.getFigure(circle)->radius,3);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,3u);

        DCMManager alias;
        auto a=alias.addFigure(FigureDescriptor::point(0,0));
        auto b=alias.addFigure(FigureDescriptor::point(4,0));
        auto center=alias.addFigure(FigureDescriptor::point(0,0));
        auto l=alias.addFigure(FigureDescriptor::line(a,b));
        auto c=alias.addFigure(FigureDescriptor::circle(center,2));
        alias.addRequirement(RequirementDescriptor::pointOnPoint(a,center));
        alias.addRequirement(RequirementDescriptor::circleRadius(c,2));
        alias.addRequirement(RequirementDescriptor::lineCircleTangent(l,c,TangencySide::POSITIVE));
        alias.setSolveMode(mode); EXPECT_FALSE(alias.solve(alias.getComponentForFigure(c)));
        EXPECT_GT(*alias.getFigure(c)->radius,0);
    }
}

TEST(DCMManagerLineCircleTangency, RejectsInvalidSidesTypesAndUndefinedLines) {
    auto d=RequirementDescriptor::lineCircleTangent(ID(1),ID(2),static_cast<TangencySide>(0));
    EXPECT_THROW(d.validate(),std::invalid_argument);
    for (auto mode:modes) {
        DCMManager m;
        auto line=m.addFigure(FigureDescriptor::line(0,0,0,0));
        auto circle=m.addFigure(FigureDescriptor::circle(0,3,3));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::lineCircleTangent(circle,line,TangencySide::POSITIVE)),std::runtime_error);
        auto id=m.addRequirement(RequirementDescriptor::lineCircleTangent(line,circle,TangencySide::POSITIVE));
        EXPECT_THROW(m.updateRequirementParam(id,0),std::invalid_argument);
        m.setSolveMode(mode); EXPECT_FALSE(m.solve(m.getComponentForFigure(circle)));
        EXPECT_EQ(m.getRequirementSystem().diagnose(),SystemStatus::SINGULAR_SYSTEM);
        EXPECT_EQ(m.getFigure(circle)->coords,(std::vector<double>{0,3}));
        EXPECT_EQ(m.getFigure(line)->coords,(std::vector<double>{0,0,0,0}));
    }
}
