#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

TEST(DCMManagerMathIntegration, ArcValidityIsMandatoryWithoutUserRequirementAndAfterRemoval) {
    for (auto mode : {SolveMode::GLOBAL,SolveMode::LOCAL,SolveMode::DRAG}) {
        DCMManager manager;
        auto arc=manager.addFigure(FigureDescriptor::arc(0,0,4,0,3,3));
        manager.setSolveMode(mode);
        auto component=manager.getComponentForFigure(arc);
        ASSERT_TRUE(component);
        EXPECT_EQ(manager.requirementCount(),0u);
        ASSERT_EQ(manager.getRequirementSystem().getFunctions().size(),1u);
        EXPECT_FALSE(manager.getRequirementSystem().getFunctions()[0]->requirementId());
        EXPECT_TRUE(manager.solve(*component));
        auto explicitId=manager.addRequirement(RequirementDescriptor::arcCenterOnPerpendicular(arc));
        manager.updateRequirementWeight(explicitId,0);
        EXPECT_TRUE(manager.solve(*manager.getComponentForFigure(arc)));
        manager.removeRequirement(explicitId);
        auto snapshot=manager.snapshot();
        manager.restoreSnapshot(snapshot);
        EXPECT_TRUE(manager.solve(*manager.getComponentForFigure(arc)));
        manager.removeFigure(arc,true);
        EXPECT_TRUE(manager.solve());
        EXPECT_TRUE(manager.getRequirementSystem().getFunctions().empty());
    }
}

TEST(DCMManagerMathIntegration, CollapsedArcFailsAndLocalScopeKeepsOtherComponentIndependent) {
    DCMManager manager;
    auto arc=manager.addFigure(FigureDescriptor::arc(0,0,0,0,2,2));
    auto line=manager.addFigure(FigureDescriptor::line(0,0,4,0));
    manager.addRequirement(RequirementDescriptor::horizontal(line));
    EXPECT_FALSE(manager.solve());
    manager.setSolveMode(SolveMode::LOCAL);
    EXPECT_TRUE(manager.solve(*manager.getComponentForFigure(line)));
    EXPECT_FALSE(manager.solve(*manager.getComponentForFigure(arc)));
}

TEST(DCMManagerMathIntegration, ZeroWeightFixAndCoincidenceAreActuallyDisabled) {
    DCMManager manager;
    auto a=manager.addFigure(FigureDescriptor::point(0,0));
    auto b=manager.addFigure(FigureDescriptor::point(4,0));
    auto fixed=RequirementDescriptor::fixPoint(a); fixed.weight=0;
    auto fixId=manager.addRequirement(fixed);
    auto coincidence=RequirementDescriptor::pointOnPoint(a,b); coincidence.weight=0;
    auto onId=manager.addRequirement(coincidence);
    manager.updatePoint({a,2,3});
    EXPECT_DOUBLE_EQ(*manager.getFigure(a)->x,2);
    EXPECT_DOUBLE_EQ(*manager.getFigure(b)->x,4);
    EXPECT_TRUE(manager.solve());
    manager.updateRequirementWeight(fixId,1);
    EXPECT_TRUE(manager.solve());
    EXPECT_DOUBLE_EQ(*manager.getFigure(a)->x,0);
    manager.updateRequirementWeight(fixId,0);
    manager.updateRequirementWeight(onId,1);
    EXPECT_TRUE(manager.solve());
    EXPECT_DOUBLE_EQ(*manager.getFigure(a)->x,*manager.getFigure(b)->x);
}

TEST(DCMManagerMathIntegration, BindingTracksRequirementRowsAndSavedFixTargetsAfterRebuild) {
    DCMManager manager;
    auto line=manager.addFigure(FigureDescriptor::line(3,0,7,0));
    auto circle=manager.addFigure(FigureDescriptor::circle(0,0,5));
    auto id=manager.addRequirement(RequirementDescriptor::lineOnCircle(line,circle));
    auto& system=manager.requirementSystem();
    ASSERT_EQ(system.getFunctions().size(),2u);
    for (const auto& row : system.getFunctions()) {
        EXPECT_EQ(row->requirementId(),id);
        EXPECT_EQ(row->mathematical()->kind(),Math::ConstraintKind::PointOnCircle);
    }
    manager.removeRequirement(id);
    manager.removeFigure(circle,true);
    EXPECT_TRUE(manager.solve());
    EXPECT_TRUE(manager.getRequirementSystem().getFunctions().empty());
}
