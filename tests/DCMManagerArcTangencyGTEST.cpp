#include "DCMManager.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;
namespace {
constexpr SolveMode modes[]={SolveMode::GLOBAL,SolveMode::LOCAL,SolveMode::DRAG};
constexpr Endpoint ends[]={Endpoint::FIRST,Endpoint::SECOND};
ID endpoint(const DCMManager& m,ID id,Endpoint e) {return m.getFigure(id)->pointIds[static_cast<std::size_t>(e)];}
void fixArc(DCMManager& m,ID arc) {
    const auto points=m.getFigure(arc)->pointIds;
    for (auto p:points) m.addRequirement(RequirementDescriptor::fixPoint(p));
}
void expectConnected(const DCMManager& m,ID a,Endpoint ae,ID b,Endpoint be) {
    const auto p=m.getFigure(endpoint(m,a,ae)).value();
    const auto q=m.getFigure(endpoint(m,b,be)).value();
    EXPECT_NEAR(*p.x,*q.x,1e-7); EXPECT_NEAR(*p.y,*q.y,1e-7);
}
void expectValidArc(const DCMManager& m,ID arc) {
    auto c=m.getFigure(arc)->coords;
    const double r1=std::hypot(c[0]-c[4],c[1]-c[5]);
    const double r2=std::hypot(c[2]-c[4],c[3]-c[5]);
    EXPECT_GT(r1,0); EXPECT_NEAR(r1,r2,1e-6);
}
}

TEST(DCMManagerArcTangency, ArcLineAllSelectedEndsAndAlreadySharedContactsHaveOneFreedom) {
    for (auto mode:modes) for (auto ae:ends) for (auto le:ends) for (bool shared:{false,true}) {
        DCMManager m;
        auto arc=m.addFigure(ae==Endpoint::FIRST ? FigureDescriptor::arc(1,0,0,1,0,0) : FigureDescriptor::arc(0,1,1,0,0,0));
        auto t=shared ? endpoint(m,arc,ae) : m.addFigure(FigureDescriptor::point(1,0));
        auto other=m.addFigure(FigureDescriptor::point(3,3));
        auto line=m.addFigure(le==Endpoint::FIRST ? FigureDescriptor::line(t,other) : FigureDescriptor::line(other,t));
        fixArc(m,arc);
        auto id=m.addRequirement(RequirementDescriptor::arcLineTangent(arc,ae,line,le));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        expectConnected(m,arc,ae,line,le); expectValidArc(m,arc);
        EXPECT_NEAR(*m.getFigure(other)->x,1,1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,1u);
        EXPECT_EQ(m.getRequirementSystem().getFunctions().size(),8u);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        auto saved=m.snapshot(); m.removeRequirement(id);
        m.restoreSnapshot(saved);
        EXPECT_EQ(m.getRequirement(id)->firstEndpoint,ae); EXPECT_EQ(m.getRequirement(id)->secondEndpoint,le);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        if (mode==SolveMode::DRAG) {
            m.updatePoint({other,5,6});
            EXPECT_NEAR(*m.getFigure(other)->x,1,1e-6);
            m.updatePoint({t,9,8}); expectConnected(m,arc,ae,line,le);
            EXPECT_NEAR(*m.getFigure(t)->x,1,1e-6);
        }
        m.removeFigure(line,true); EXPECT_FALSE(m.hasRequirement(id));
    }
}

TEST(DCMManagerArcTangency, ArcArcBothDirectionsAndAllEndpointSelectionsPreserveInternalConditions) {
    for (auto mode:modes) for (auto first:ends) for (auto second:ends) for (bool opposite:{false,true}) for (bool shared:{false,true}) {
        DCMManager m;
        auto a=m.addFigure(first==Endpoint::FIRST ? FigureDescriptor::arc(1,0,0,1,0,0) : FigureDescriptor::arc(0,1,1,0,0,0));
        // The second center can move onto the common radius line.
        auto t=shared ? endpoint(m,a,first) : m.addFigure(FigureDescriptor::point(1,0));
        auto other=m.addFigure(FigureDescriptor::point(opposite?3:-1,2));
        auto center=m.addFigure(FigureDescriptor::point(opposite?3:-1,0.5));
        auto b=m.addFigure(second==Endpoint::FIRST
            ? FigureDescriptor::arc(t,other,center) : FigureDescriptor::arc(other,t,center));
        fixArc(m,a);
        auto id=m.addRequirement(RequirementDescriptor::arcArcTangent(a,first,b,second));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        expectConnected(m,a,first,b,second); expectValidArc(m,a); expectValidArc(m,b);
        EXPECT_NEAR(m.getFigure(b)->coords[5],0,1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,2u);
        EXPECT_EQ(m.getRequirementSystem().getFunctions().size(),9u);
        auto saved=m.snapshot(); m.removeRequirement(id);
        if (!shared) EXPECT_NE(m.getComponentForFigure(a),m.getComponentForFigure(b));
        m.restoreSnapshot(saved); ASSERT_TRUE(m.solve(m.getComponentForFigure(b)));
        m.removeFigure(b,true); EXPECT_FALSE(m.hasRequirement(id));
    }
}

TEST(DCMManagerArcTangency, ZeroLengthLineAndEitherZeroArcRadiusStayUndefinedUntilDisabled) {
    for (auto mode:modes) for (bool twoArcs:{false,true}) {
        DCMManager m;
        auto a=m.addFigure(FigureDescriptor::arc(1,0,0,1,0,0));
        auto b=m.addFigure(twoArcs ? FigureDescriptor::arc(1,0,2,0,1,0) : FigureDescriptor::line(1,0,1,0));
        fixArc(m,a);
        auto d=twoArcs ? RequirementDescriptor::arcArcTangent(a,Endpoint::FIRST,b,Endpoint::FIRST)
            : RequirementDescriptor::arcLineTangent(a,Endpoint::FIRST,b,Endpoint::FIRST);
        auto id=m.addRequirement(d); m.setSolveMode(mode);
        EXPECT_FALSE(m.solve(m.getComponentForFigure(b)));
        EXPECT_EQ(m.getRequirementSystem().diagnose(),SystemStatus::SINGULAR_SYSTEM);
        if (!twoArcs) {
            m.updateRequirementWeight(id,0);
            EXPECT_TRUE(m.solve(m.getComponentForFigure(b)));
        }
    }
}

TEST(DCMManagerArcTangency, WeightAndEndpointEditsRebuildAliasesAndSnapshotsRestoreThem) {
    for (auto mode:modes) {
        DCMManager m;
        auto arc=m.addFigure(FigureDescriptor::arc(1,0,0,1,0,0));
        auto line=m.addFigure(FigureDescriptor::line(1,0,1,3));
        fixArc(m,arc);
        auto id=m.addRequirement(RequirementDescriptor::arcLineTangent(arc,Endpoint::FIRST,line,Endpoint::FIRST));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        auto saved=m.snapshot(); m.updateRequirementWeight(id,0);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,4u);
        m.updateLine({line,4,5,7,8}); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        EXPECT_EQ(m.getFigure(line)->coords,(std::vector<double>{4,5,7,8}));
        m.updateRequirementWeight(id,2); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        expectConnected(m,arc,Endpoint::FIRST,line,Endpoint::FIRST);
        m.updateArcTangencyEndpoints(id,Endpoint::SECOND,Endpoint::SECOND);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        expectConnected(m,arc,Endpoint::SECOND,line,Endpoint::SECOND);
        EXPECT_NEAR(m.getFigure(line)->coords[1],1,1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,1u);
        m.restoreSnapshot(saved); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
        expectConnected(m,arc,Endpoint::FIRST,line,Endpoint::FIRST);
        EXPECT_EQ(m.getRequirement(id)->firstEndpoint,Endpoint::FIRST);
        m.removeRequirement(id); m.updateLine({line,5,6,7,8});
        EXPECT_EQ(m.getFigure(arc)->coords,(std::vector<double>{1,0,0,1,0,0}));
    }
}

TEST(DCMManagerArcTangency, RejectsMissingEndsWrongTypesZeroRadiiAndIncompatibleFixations) {
    auto d=RequirementDescriptor::arcLineTangent(ID(1),Endpoint::FIRST,ID(2),Endpoint::SECOND);
    d.firstEndpoint.reset(); EXPECT_THROW(d.validate(),std::invalid_argument);
    d.firstEndpoint=static_cast<Endpoint>(3); EXPECT_THROW(d.validate(),std::invalid_argument);
    auto plain=RequirementDescriptor::horizontal(ID(1)); plain.firstEndpoint=Endpoint::FIRST;
    EXPECT_THROW(plain.validate(),std::invalid_argument);
    for (auto mode:modes) for (bool zeroRadius:{false,true}) {
        DCMManager m;
        auto arc=m.addFigure(FigureDescriptor::arc(zeroRadius?0:1,0,0,1,0,0));
        auto line=m.addFigure(FigureDescriptor::line(zeroRadius?0:1,0,2,2));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::arcLineTangent(line,Endpoint::FIRST,arc,Endpoint::FIRST)),std::runtime_error);
        fixArc(m,arc); m.addRequirement(RequirementDescriptor::fixLine(line));
        auto id=m.addRequirement(RequirementDescriptor::arcLineTangent(arc,Endpoint::FIRST,line,Endpoint::FIRST));
        m.setSolveMode(mode); auto before=m.getFigure(line)->coords;
        EXPECT_FALSE(m.solve(m.getComponentForFigure(line)));
        EXPECT_EQ(m.getFigure(line)->coords,before);
        EXPECT_THROW(m.updateArcTangencyEndpoints(id,static_cast<Endpoint>(3),Endpoint::FIRST),std::invalid_argument);
        EXPECT_EQ(m.getRequirement(id)->firstEndpoint,Endpoint::FIRST);
        if (zeroRadius) EXPECT_EQ(m.getRequirementSystem().diagnose(),SystemStatus::SINGULAR_SYSTEM);
    }
}

TEST(DCMManagerArcTangency, LocalSolveKeepsUnrelatedUndefinedArcOutsideTheScope) {
    DCMManager m;
    auto arc=m.addFigure(FigureDescriptor::arc(1,0,0,1,0,0));
    auto line=m.addFigure(FigureDescriptor::line(1,0,2,3));
    auto bad=m.addFigure(FigureDescriptor::arc(5,5,5,5,6,6));
    fixArc(m,arc);
    m.addRequirement(RequirementDescriptor::arcLineTangent(arc,Endpoint::FIRST,line,Endpoint::FIRST));
    m.setSolveMode(SolveMode::LOCAL); ASSERT_TRUE(m.solve(m.getComponentForFigure(line)));
    EXPECT_EQ(m.getFigure(bad)->coords,(std::vector<double>{5,5,5,5,6,6}));
    m.setSolveMode(SolveMode::GLOBAL); EXPECT_FALSE(m.solve());
}
