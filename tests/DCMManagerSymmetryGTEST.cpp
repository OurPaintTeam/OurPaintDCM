#include "DCMManager.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;
namespace {
constexpr SolveMode modes[]={SolveMode::GLOBAL,SolveMode::LOCAL,SolveMode::DRAG};
void point(const DCMManager& m,ID id,double x,double y) {
    auto p=m.getFigure(id).value(); EXPECT_NEAR(*p.x,x,1e-6); EXPECT_NEAR(*p.y,y,1e-6);
}
}

TEST(DCMManagerSymmetry, FixedTiltedAxisAndFixedPointReflectInAllModes) {
    for (auto mode:modes) {
        DCMManager m;
        auto axis=m.addFigure(FigureDescriptor::line(0,0,4,4));
        auto p=m.addFigure(FigureDescriptor::point(1,3));
        auto q=m.addFigure(FigureDescriptor::point(-5,2));
        m.addRequirement(RequirementDescriptor::fixLine(axis));
        m.addRequirement(RequirementDescriptor::fixPoint(p));
        auto id=m.addRequirement(RequirementDescriptor::symmetricAboutLine(p,q,axis));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(q)));
        point(m,q,3,1); point(m,p,1,3);
        EXPECT_EQ(m.getFigure(axis)->coords,(std::vector<double>{0,0,4,4}));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,0u);
        const auto rows=m.getRequirementSystem().getFunctions();
        EXPECT_EQ(rows[6]->requirementId(),id); EXPECT_EQ(rows[7]->requirementId(),id);
        auto saved=m.snapshot(); m.removeRequirement(id);
        EXPECT_NE(m.getComponentForFigure(q),m.getComponentForFigure(axis));
        m.restoreSnapshot(saved); ASSERT_TRUE(m.solve(m.getComponentForFigure(q)));
        m.removeFigure(axis,true); EXPECT_FALSE(m.hasRequirement(id));
    }
}

TEST(DCMManagerSymmetry, FreeAxisAndCoincidentPointsKeepCorrectFreedom) {
    for (auto mode:modes) {
        DCMManager m;
        auto p=m.addFigure(FigureDescriptor::point(2,4));
        auto q=m.addFigure(FigureDescriptor::point(2,-2));
        auto axis=m.addFigure(FigureDescriptor::line(-3,0,5,2));
        m.addRequirement(RequirementDescriptor::fixPoint(p));
        m.addRequirement(RequirementDescriptor::fixPoint(q));
        m.addRequirement(RequirementDescriptor::symmetricAboutLine(p,q,axis));
        m.setSolveMode(mode); ASSERT_TRUE(m.solve(m.getComponentForFigure(axis)));
        auto l=m.getFigure(axis)->coords;
        EXPECT_NEAR(l[1],1,1e-6); EXPECT_NEAR(l[3],1,1e-6);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,2u);

        DCMManager alias;
        auto a=alias.addFigure(FigureDescriptor::point(2,5));
        auto b=alias.addFigure(FigureDescriptor::point(2,5));
        auto line=alias.addFigure(FigureDescriptor::line(0,0,4,0));
        alias.addRequirement(RequirementDescriptor::fixLine(line));
        alias.addRequirement(RequirementDescriptor::pointOnPoint(a,b));
        alias.addRequirement(RequirementDescriptor::symmetricAboutLine(a,b,line));
        alias.setSolveMode(mode); ASSERT_TRUE(alias.solve(alias.getComponentForFigure(a)));
        point(alias,a,2,0); point(alias,b,2,0);
        EXPECT_EQ(alias.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,1u);
    }
}

TEST(DCMManagerSymmetry, CoordinateAndCentralVariantsSupportWeightsEditsAndSnapshots) {
    for (auto mode:modes) for (int variant=0;variant<3;++variant) {
        DCMManager m;
        auto p=m.addFigure(FigureDescriptor::point(1,3));
        auto q=m.addFigure(FigureDescriptor::point(5,-6));
        auto center=variant==2 ? m.addFigure(FigureDescriptor::point(2,2)) : ID{};
        if (variant==2) m.addRequirement(RequirementDescriptor::fixPoint(center));
        auto d=variant==0 ? RequirementDescriptor::symmetricAboutHorizontal(p,q,2)
            : variant==1 ? RequirementDescriptor::symmetricAboutVertical(p,q,2)
            : RequirementDescriptor::symmetricAboutPoint(p,q,center);
        auto id=m.addRequirement(d); m.setSolveMode(mode);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,2u);
        auto saved=m.snapshot(); m.updateRequirementWeight(id,0);
        m.updatePoint({q,8,9}); ASSERT_TRUE(m.solve(m.getComponentForFigure(q)));
        point(m,q,8,9);
        EXPECT_EQ(m.getRequirementSystem().diagnoseDetailed().degreesOfFreedom,4u);
        m.restoreSnapshot(saved); m.updateRequirementWeight(id,2);
        if (variant<2) m.updateRequirementParam(id,-3);
        ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
        auto a=m.getFigure(p).value(), b=m.getFigure(q).value();
        if (variant==0) { EXPECT_NEAR(*a.x,*b.x,1e-6); EXPECT_NEAR((*a.y+*b.y)/2,-3,1e-6); }
        if (variant==1) { EXPECT_NEAR(*a.y,*b.y,1e-6); EXPECT_NEAR((*a.x+*b.x)/2,-3,1e-6); }
        if (variant==2) { EXPECT_NEAR((*a.x+*b.x)/2,2,1e-6); EXPECT_NEAR((*a.y+*b.y)/2,2,1e-6); }
        if (mode==SolveMode::DRAG) {
            m.updatePoint({q,7,8});
            ASSERT_TRUE(m.solve(m.getComponentForFigure(p)));
        }
    }
}

TEST(DCMManagerSymmetry, RejectsBadDescriptorsUndefinedAxesAndIncompatibleFixes) {
    EXPECT_THROW(RequirementDescriptor::symmetricAboutHorizontal(ID(1),ID(2),
        std::numeric_limits<double>::infinity()).validate(),std::invalid_argument);
    auto bad=RequirementDescriptor::symmetricAboutLine(ID(1),ID(2),ID(3));
    bad.objectIds.pop_back(); EXPECT_THROW(bad.validate(),std::invalid_argument);
    for (auto mode:modes) for (bool collapsed:{false,true}) {
        DCMManager m;
        auto line=m.addFigure(FigureDescriptor::line(0,0,collapsed?0:4,0));
        auto p=m.addFigure(FigureDescriptor::point(1,2));
        auto q=m.addFigure(FigureDescriptor::point(3,2));
        EXPECT_THROW(m.addRequirement(RequirementDescriptor::symmetricAboutLine(p,line,q)),std::runtime_error);
        m.addRequirement(RequirementDescriptor::fixLine(line));
        m.addRequirement(RequirementDescriptor::fixPoint(p));
        m.addRequirement(RequirementDescriptor::fixPoint(q));
        m.addRequirement(RequirementDescriptor::symmetricAboutLine(p,q,line));
        m.setSolveMode(mode); EXPECT_FALSE(m.solve(m.getComponentForFigure(p)));
        point(m,p,1,2); point(m,q,3,2);
        if (collapsed) EXPECT_EQ(m.getRequirementSystem().diagnose(),SystemStatus::SINGULAR_SYSTEM);
    }
}
