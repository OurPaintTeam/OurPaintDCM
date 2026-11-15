#include "DCMManager.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;
namespace {
constexpr SolveMode modes[]={SolveMode::GLOBAL,SolveMode::LOCAL,SolveMode::DRAG};
constexpr CircleTangencyKind kinds[]={CircleTangencyKind::EXTERNAL,
    CircleTangencyKind::FIRST_CONTAINS_SECOND,CircleTangencyKind::SECOND_CONTAINS_FIRST};
void fixArc(DCMManager& m,ID arc) {
    const auto points=m.getFigure(arc)->pointIds;
    for (auto p:points) m.addRequirement(RequirementDescriptor::fixPoint(p));
}
void fixRadius(DCMManager& m,ID arc,double r) {
    const auto ids=m.getFigure(arc)->pointIds;
    m.addRequirement(RequirementDescriptor::pointPointDist(ids[0],ids[2],r));
}
void expectValidArc(const DCMManager& m,ID arc) {
    const auto c=m.getFigure(arc)->coords;
    const double r=std::hypot(c[0]-c[4],c[1]-c[5]);
    EXPECT_GT(r,0); EXPECT_NEAR(r,std::hypot(c[2]-c[4],c[3]-c[5]),1e-6);
}
double centerDistance(const DCMManager& m,ID a,ID b) {
    const auto x=m.getFigure(a)->coords,y=m.getFigure(b)->coords;
    const auto i=x.size()==6 ? 4 : 0,j=y.size()==6 ? 4 : 0;
    return std::hypot(x[i]-y[j],x[i+1]-y[j+1]);
}
double signedDistance(const DCMManager& m,ID line,ID arc) {
    const auto l=m.getFigure(line)->coords,c=m.getFigure(arc)->coords;
    const double dx=l[2]-l[0],dy=l[3]-l[1];
    return ((c[5]-l[1])*dx-(c[4]-l[0])*dy)/std::hypot(dx,dy);
}
}

TEST(DCMManagerArcTangency, FixedGeometryAllowsContactOutsideArcAndSegmentWithoutJoiningEnds) {
    for (auto mode:modes) for (auto side:{TangencySide::POSITIVE,TangencySide::NEGATIVE}) for (bool generic:{false,true}) {
        DCMManager m;
        auto arc=m.addFigure(FigureDescriptor::arc(3,4,-3,4,0,0));
        const double y=-5*static_cast<double>(side);
        auto line=m.addFigure(FigureDescriptor::line(10,y,20,y));
        fixArc(m,arc); m.addRequirement(RequirementDescriptor::fixLine(line));
        const auto beforeArc=m.getFigure(arc)->coords,beforeLine=m.getFigure(line)->coords;
        m.addRequirement(generic ? RequirementDescriptor::lineCircleTangent(line,arc,side)
            : RequirementDescriptor::arcLineTangent(arc,line,side));
        EXPECT_EQ(m.getFigure(arc)->coords,beforeArc); EXPECT_EQ(m.getFigure(line)->coords,beforeLine);
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        EXPECT_EQ(m.getFigure(arc)->coords,beforeArc); EXPECT_EQ(m.getFigure(line)->coords,beforeLine);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,0u);
    }
}

TEST(DCMManagerArcTangency, ArcLineSolvesAndPreservesSideWeightSnapshotsAndIndependentEnds) {
    for (auto mode:modes) {
        DCMManager m;
        auto arc=m.addFigure(FigureDescriptor::arc(3,4,-3,4,0,0));
        auto line=m.addFigure(FigureDescriptor::line(10,-8,20,-7));
        fixArc(m,arc);
        auto id=m.addRequirement(RequirementDescriptor::arcLineTangent(arc,line,TangencySide::POSITIVE));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        EXPECT_NEAR(signedDistance(m,line,arc),5,1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,3u);
        auto saved=m.snapshot();
        m.updateLineCircleTangencySide(id,TangencySide::NEGATIVE);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        EXPECT_NEAR(signedDistance(m,line,arc),-5,1e-6);
        m.updateRequirementWeight(id,0);m.updateLine({line,10,8,20,8});
        ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        EXPECT_EQ(m.getFigure(line)->coords,(std::vector<double>{10,8,20,8}));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,4u);
        m.updateRequirementWeight(id,2);ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        EXPECT_NEAR(signedDistance(m,line,arc),-5,1e-6);
        m.restoreSnapshot(saved);EXPECT_EQ(m.getRequirement(id)->param,1);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        if (mode==SolveMode::DRAG) {
            m.updateLine({line,10,-9,20,-9});
            EXPECT_NEAR(signedDistance(m,line,arc),5,1e-6);
        }
        m.removeRequirement(id);EXPECT_NE(m.getComponentForFigure(arc),m.getComponentForFigure(line));
        m.updateLine({line,10,12,20,12});
        EXPECT_EQ(m.getFigure(arc)->coords,(std::vector<double>{3,4,-3,4,0,0}));
        m.restoreSnapshot(saved);m.removeFigure(line,true);EXPECT_FALSE(m.hasRequirement(id));
    }
}

TEST(DCMManagerArcTangency, ArcArcEveryBranchSolvesWithoutSelectingContactEnds) {
    for (auto mode:modes) for (auto kind:kinds) for (bool generic:{false,true}) {
        SCOPED_TRACE(static_cast<int>(mode));
        SCOPED_TRACE(static_cast<int>(kind));
        DCMManager m;
        const double r1=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST ? 2 : 5;
        const double r2=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST ? 5 : 2;
        auto a=m.addFigure(FigureDescriptor::arc(0,r1,-r1,0,0,0));
        auto b=m.addFigure(FigureDescriptor::arc(9,4+r2,9-r2,4,9,4));
        fixArc(m,a);fixRadius(m,b,r2);
        auto id=m.addRequirement(generic ? RequirementDescriptor::circleCircleTangent(a,b,kind)
            : RequirementDescriptor::arcArcTangent(a,b,kind));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        EXPECT_NEAR(centerDistance(m,a,b),kind==CircleTangencyKind::EXTERNAL ? r1+r2 : std::abs(r1-r2),1e-6);
        expectValidArc(m,b);
        EXPECT_EQ(m.getFigure(a)->coords,(std::vector<double>{0,r1,-r1,0,0,0}));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,3u);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        auto saved=m.snapshot();
        auto other=kind==CircleTangencyKind::EXTERNAL ? CircleTangencyKind::FIRST_CONTAINS_SECOND : CircleTangencyKind::EXTERNAL;
        m.updateCircleCircleTangencyKind(id,other);ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        EXPECT_NEAR(centerDistance(m,a,b),other==CircleTangencyKind::EXTERNAL ? r1+r2 : std::abs(r1-r2),1e-6);
        m.restoreSnapshot(saved);EXPECT_EQ(m.getRequirement(id)->param,static_cast<double>(kind));
        ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        m.removeRequirement(id);EXPECT_NE(m.getComponentForFigure(a),m.getComponentForFigure(b));
    }
}

TEST(DCMManagerArcTangency, MixedCircleArcPreservesContainmentInEitherObjectOrder) {
    for (auto mode:modes) for (auto kind:kinds) for (bool arcFirst:{false,true}) {
        DCMManager m;
        const double firstRadius=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST ? 2 : 5;
        const double secondRadius=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST ? 5 : 2;
        const double cr=arcFirst ? secondRadius : firstRadius, ar=arcFirst ? firstRadius : secondRadius;
        auto circle=m.addFigure(FigureDescriptor::circle(0,0,cr));
        auto arc=m.addFigure(FigureDescriptor::arc(9,4+ar,9-ar,4,9,4));
        m.addRequirement(RequirementDescriptor::fixCircle(circle));fixRadius(m,arc,ar);
        auto id=m.addRequirement(RequirementDescriptor::circleArcTangent(arcFirst?arc:circle,arcFirst?circle:arc,kind));
        m.setSolveMode(mode);ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
        EXPECT_NEAR(centerDistance(m,circle,arc),kind==CircleTangencyKind::EXTERNAL ? cr+ar : std::abs(cr-ar),1e-6);
        expectValidArc(m,arc);EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,3u);
        EXPECT_EQ(m.getRequirement(id)->param,static_cast<double>(kind));
        auto saved=m.snapshot();m.removeRequirement(id);m.restoreSnapshot(saved);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
    }
}

TEST(DCMManagerArcTangency, FixedArcsAllowContactOutsideBothSpansForEveryBranch) {
    for (auto mode:modes) for (auto kind:kinds) {
        DCMManager m;
        const double r1=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST?2:5;
        const double r2=kind==CircleTangencyKind::SECOND_CONTAINS_FIRST?5:2;
        const double d=kind==CircleTangencyKind::EXTERNAL?r1+r2:std::abs(r1-r2);
        auto a=m.addFigure(FigureDescriptor::arc(0,r1,-r1,0,0,0));
        auto b=m.addFigure(FigureDescriptor::arc(d,r2,d,-r2,d,0));
        fixArc(m,a);fixArc(m,b);
        const auto before=m.getFigure(b)->coords;
        m.addRequirement(RequirementDescriptor::arcArcTangent(a,b,kind));
        m.setSolveMode(mode);ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        EXPECT_EQ(m.getFigure(b)->coords,before);
    }
}

TEST(DCMManagerArcTangency, TangencyRemovesOneFreedomAndOnlyExplicitCoincidenceJoinsPoints) {
    for (int otherType=0;otherType<3;++otherType) {
        DCMManager m;
        auto a=m.addFigure(FigureDescriptor::arc(3,4,-3,4,0,0));
        auto b=m.addFigure(otherType==0 ? FigureDescriptor::line(10,-5,20,-5)
            : otherType==1 ? FigureDescriptor::arc(7,2,9,0,7,0) : FigureDescriptor::circle(7,0,2));
        const auto initial=m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom;
        auto id=m.addRequirement(otherType==0 ? RequirementDescriptor::arcLineTangent(a,b,TangencySide::POSITIVE)
            : RequirementDescriptor::circleArcTangent(a,b,CircleTangencyKind::EXTERNAL));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,initial.value()-1);
        m.removeRequirement(id);EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,initial);
    }
    DCMManager m;
    auto a=m.addFigure(FigureDescriptor::arc(1,0,0,1,0,0));
    auto line=m.addFigure(FigureDescriptor::line(1,0,1,3));
    fixArc(m,a);
    m.addRequirement(RequirementDescriptor::arcLineTangent(a,line,TangencySide::POSITIVE));
    auto coincidence=m.addRequirement(RequirementDescriptor::pointOnPoint(m.getFigure(a)->pointIds[0],m.getFigure(line)->pointIds[0]));
    ASSERT_TRUE(m.solve());
    EXPECT_NEAR(m.getFigure(line)->coords[0],1,1e-7);
    EXPECT_NEAR(m.getFigure(line)->coords[1],0,1e-7);
    m.removeRequirement(coincidence);EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,3u);
}

TEST(DCMManagerArcTangency, InvalidParametersTypesAndFixedUnsatisfiedGeometryNeverPass) {
    auto d=RequirementDescriptor::arcLineTangent(ID(1),ID(2),TangencySide::POSITIVE);
    d.param.reset();EXPECT_THROW(d.validate(),std::invalid_argument);
    d.param=0;EXPECT_THROW(d.validate(),std::invalid_argument);
    EXPECT_THROW(RequirementDescriptor::arcArcTangent(ID(1),ID(2),static_cast<CircleTangencyKind>(3)).validate(),std::invalid_argument);
    for (auto mode:modes) for (int invalid=0;invalid<5;++invalid) {
        DCMManager m;
        auto a=m.addFigure(invalid==1 ? FigureDescriptor::arc(0,0,0,1,0,0)
            : invalid==2 ? FigureDescriptor::arc(1,0,1,0,0,0) : FigureDescriptor::arc(1,0,0,1,0,0));
        auto b=m.addFigure(invalid>=3 ? FigureDescriptor::arc(0,1,-1,0,0,0)
            : invalid==0 ? FigureDescriptor::line(1,0,1,0) : FigureDescriptor::line(5,0,5,3));
        fixArc(m,a);if(invalid>=3) fixArc(m,b);else m.addRequirement(RequirementDescriptor::fixLine(b));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcLineTangent(b,a,TangencySide::POSITIVE)),std::runtime_error);
        auto desc=invalid>=3 ? RequirementDescriptor::arcArcTangent(a,b,invalid==3 ? CircleTangencyKind::EXTERNAL : CircleTangencyKind::FIRST_CONTAINS_SECOND)
            : RequirementDescriptor::arcLineTangent(a,b,TangencySide::POSITIVE);
        auto id=m.addRequirement(desc);const auto before=m.getFigure(b)->coords;
        m.setSolveMode(mode);EXPECT_FALSE(m.solve(m.getComponentForFigure(b)));
        EXPECT_EQ(m.getFigure(b)->coords,before);
        EXPECT_THROW(m.updateRequirementParam(id,0.5),std::invalid_argument);EXPECT_EQ(m.getRequirement(id)->param,desc.param);
        if (invalid==0) {m.updateRequirementWeight(id,0);EXPECT_TRUE(m.solve(m.getComponentForFigure(b)));}
    }
}

TEST(DCMManagerArcTangency, LocalSolveKeepsUnrelatedUndefinedArcOutsideScope) {
    DCMManager m;
    auto arc=m.addFigure(FigureDescriptor::arc(1,0,0,1,0,0));
    auto line=m.addFigure(FigureDescriptor::line(1,0,2,3));
    auto bad=m.addFigure(FigureDescriptor::arc(5,5,5,5,6,6));
    fixArc(m,arc);
    m.addRequirement(RequirementDescriptor::arcLineTangent(arc,line,TangencySide::POSITIVE));
    m.setSolveMode(SolveMode::LOCAL);ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
    EXPECT_EQ(m.getFigure(bad)->coords,(std::vector<double>{5,5,5,5,6,6}));
    m.setSolveMode(SolveMode::GLOBAL);EXPECT_FALSE(m.solve());
}

TEST(DCMManagerArcTangency, CoincidentCentersRecoverAndArcDraggingKeepsSupportingCircleContact) {
    for (auto mode:modes) for (bool mixed:{false,true}) {
        DCMManager m;
        auto a=m.addFigure(mixed ? FigureDescriptor::circle(0,0,3) : FigureDescriptor::arc(0,3,-3,0,0,0));
        auto b=m.addFigure(FigureDescriptor::arc(0,2,-2,0,0,0));
        if (mixed) m.addRequirement(RequirementDescriptor::fixCircle(a));else fixArc(m,a);
        fixRadius(m,b,2);
        m.addRequirement(RequirementDescriptor::circleArcTangent(a,b,CircleTangencyKind::EXTERNAL));
        m.setSolveMode(mode);ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        EXPECT_NEAR(centerDistance(m,a,b),5,1e-6);expectValidArc(m,b);
        if (mode==SolveMode::DRAG) {
            m.updateArc({b,8,6,6,4,8,4});
            EXPECT_NEAR(centerDistance(m,a,b),5,1e-6);expectValidArc(m,b);
        }
    }
}

TEST(DCMManagerArcTangency, AliasedCenterAndFixedIncompatibleContactRemainConsistent) {
    for (auto mode:modes) {
        DCMManager m;
        auto arc=m.addFigure(FigureDescriptor::arc(3,4,-3,4,0,0));
        auto anchor=m.addFigure(FigureDescriptor::point(0,0));
        const auto center=m.getFigure(arc)->pointIds[2];
        m.addRequirement(RequirementDescriptor::pointOnPoint(center,anchor));
        m.addRequirement(RequirementDescriptor::fixPoint(anchor));
        fixRadius(m,arc,5);
        auto line=m.addFigure(FigureDescriptor::line(10,-8,20,-8));
        m.addRequirement(RequirementDescriptor::arcLineTangent(arc,line,TangencySide::POSITIVE));
        m.setSolveMode(mode);ASSERT_TRUE(m.solve(m.getComponentForFigure(arc)));
        EXPECT_NEAR(signedDistance(m,line,arc),5,1e-6);
        EXPECT_EQ(*m.getFigure(center)->x,0);EXPECT_EQ(*m.getFigure(center)->y,0);
        expectValidArc(m,arc);

        DCMManager fixed;
        auto a=fixed.addFigure(FigureDescriptor::arc(3,4,-3,4,0,0));
        auto l=fixed.addFigure(FigureDescriptor::line(10,-8,20,-8));
        fixArc(fixed,a);fixed.addRequirement(RequirementDescriptor::fixLine(l));
        fixed.addRequirement(RequirementDescriptor::arcLineTangent(a,l,TangencySide::POSITIVE));
        const auto before=fixed.getFigure(l)->coords;
        fixed.setSolveMode(mode);EXPECT_FALSE(fixed.solve(fixed.getComponentForFigure(l)));
        EXPECT_EQ(fixed.getFigure(l)->coords,before);
    }
}
