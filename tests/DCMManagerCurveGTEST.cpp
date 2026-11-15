#include "DCMManager.h"
#include <gtest/gtest.h>
#include <limits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
FigureDescriptor bezier() {
    return FigureDescriptor::curve(3, std::vector<double>{0, 0, 0, 2, 2, 2, 2, 0},
                                    {1, 1, 1, 1}, {0, 0, 0, 0, 1, 1, 1, 1});
}
}

TEST(DCMManagerCurveTest, CreateReadAndUpdateControlPoints) {
    DCMManager manager;
    const auto id = manager.addFigure(bezier());
    const auto descriptor = manager.getFigure(id);
    ASSERT_TRUE(descriptor);
    EXPECT_EQ(descriptor->type, FigureType::ET_CURVE2D);
    EXPECT_EQ(descriptor->degree, 3u);
    EXPECT_EQ(descriptor->coords, bezier().coords);
    EXPECT_EQ(descriptor->weights, bezier().weights);
    EXPECT_EQ(descriptor->knots, bezier().knots);
    EXPECT_FALSE(descriptor->periodic);
    EXPECT_EQ(manager.figureCount(), 5u);
    EXPECT_EQ(manager.getAllFigures().size(), 5u);
    EXPECT_EQ(manager.getAllCurves().size(), 1u);
    EXPECT_EQ(manager.getComponentCount(), 1u);
    EXPECT_EQ(manager.requirementCount(), 0u);
    EXPECT_EQ(manager.getRequirementSystem().getFunctions().size(), 0u);
    ASSERT_EQ(descriptor->pointIds.size(), 4u);
    EXPECT_TRUE(manager.updatePoint({descriptor->pointIds[1], 0, 4}));
    EXPECT_DOUBLE_EQ(manager.getFigure(id)->coords[3], 4);
    const auto p = manager.getStorage().evaluateCurve(id, 0.5);
    EXPECT_DOUBLE_EQ(p.x(), 1);
    EXPECT_DOUBLE_EQ(p.y(), 2.25);
    EXPECT_TRUE(manager.updateFigure(FigureUpdateDescriptor::curve(id, bezier().coords)));
    EXPECT_EQ(manager.getFigure(id)->coords, bezier().coords);
}

TEST(DCMManagerCurveTest, ExistingControlsAndExplicitIdSurviveSnapshots) {
    DCMManager manager;
    const auto a = manager.addFigure(FigureDescriptor::point(1, 2));
    const auto b = manager.addFigure(FigureDescriptor::point(3, 4));
    const auto line = manager.addFigure(FigureDescriptor::line(a, b));
    auto descriptor = FigureDescriptor::curve(2, std::vector<ID>{a, b, a}, {1, 2, 1}, {0, 0, 0, 1, 1, 1});
    descriptor.id = ID(100);
    const auto id = manager.addFigure(descriptor);
    const auto state = manager.snapshot();
    EXPECT_TRUE(manager.updatePoint({a, 8, 9}));
    manager.removeFigure(id);
    manager.restoreSnapshot(state);
    ASSERT_TRUE(manager.getFigure(id));
    EXPECT_EQ(manager.getFigure(id)->pointIds, descriptor.pointIds);
    EXPECT_EQ(manager.getFigure(id)->weights, descriptor.weights);
    EXPECT_EQ(manager.getFigure(id)->knots, descriptor.knots);
    EXPECT_EQ(manager.getFigure(id)->coords, (std::vector<double>{1, 2, 3, 4, 1, 2}));
    EXPECT_EQ(manager.figureCount(), 4u);
    EXPECT_EQ(manager.getComponentForFigure(line), manager.getComponentForFigure(id));
    const auto next = manager.addFigure(FigureDescriptor::point(0, 0));
    EXPECT_EQ(next, ID(101));
}

TEST(DCMManagerCurveTest, PeriodicDataSurviveSnapshotAndClear) {
    DCMManager manager;
    const auto id = manager.addFigure(FigureDescriptor::curve(2,
        std::vector<double>{0, 0, 2, 0, 2, 2, 0, 2}, {1, 2, 1, 2},
        {-2, -1, 0, 1, 2, 3, 4, 5, 6}, true));
    const auto state = manager.snapshot();
    manager.clear();
    EXPECT_TRUE(manager.getAllCurves().empty());
    manager.restoreSnapshot(state);
    const auto descriptor = manager.getFigure(id);
    ASSERT_TRUE(descriptor);
    EXPECT_TRUE(descriptor->periodic);
    EXPECT_EQ(descriptor->degree, 2u);
    const auto p = manager.getStorage().evaluateCurve(id, 0.5);
    const auto wrapped = manager.getStorage().evaluateCurve(id, 4.5);
    EXPECT_DOUBLE_EQ(p.x(), wrapped.x());
    EXPECT_DOUBLE_EQ(p.y(), wrapped.y());
}

TEST(DCMManagerCurveTest, RemovingCurvePreservesControlsAndSplitsComponents) {
    DCMManager manager;
    const auto id = manager.addFigure(bezier());
    const auto controls = manager.getFigure(id)->pointIds;
    EXPECT_THROW(manager.removeFigure(controls[0]), std::runtime_error);
    manager.removeFigure(id);
    EXPECT_FALSE(manager.hasFigure(id));
    EXPECT_EQ(manager.figureCount(), 4u);
    EXPECT_EQ(manager.getComponentCount(), 4u);
    EXPECT_TRUE(manager.getAllCurves().empty());
    for (auto point : controls) EXPECT_TRUE(manager.hasFigure(point));
}

TEST(DCMManagerCurveTest, CascadeRemovesCurveAndItsControls) {
    DCMManager manager;
    const auto id = manager.addFigure(bezier());
    manager.removeFigure(id, true);
    EXPECT_EQ(manager.figureCount(), 0u);
    EXPECT_EQ(manager.getComponentCount(), 0u);
    EXPECT_TRUE(manager.getAllCurves().empty());
}

TEST(DCMManagerCurveTest, InvalidDataAreRejectedBeforeCreatingPoints) {
    DCMManager manager;
    const auto original = bezier();
    std::vector<FigureDescriptor> invalid;
    auto d = original; d.degree = 0; invalid.push_back(d);
    d = original; d.degree = 4; invalid.push_back(d);
    d = original; d.coords.pop_back(); invalid.push_back(d);
    d = original; d.weights.pop_back(); invalid.push_back(d);
    d = original; d.weights[0] = 0; invalid.push_back(d);
    d = original; d.weights[0] = -1; invalid.push_back(d);
    d = original; d.knots.pop_back(); invalid.push_back(d);
    d = original; d.knots[4] = -1; invalid.push_back(d);
    d = original; d.knots.assign(8, 0); invalid.push_back(d);
    d = original; d.periodic = true; invalid.push_back(d);
    for (double bad : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        d = original; d.coords[0] = bad; invalid.push_back(d);
        d = original; d.weights[0] = bad; invalid.push_back(d);
        d = original; d.knots[0] = bad; invalid.push_back(d);
    }
    invalid.push_back(FigureDescriptor::curve(2, std::vector<double>{0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0},
        {1, 1, 1, 1, 1, 1}, {0, 0, 0, 1, 1, 1, 2, 2, 2}));
    invalid.push_back(FigureDescriptor::curve(2, std::vector<double>{0, 0, 1, 0, 1, 1, 0, 1},
        {1, 1, 1, 1}, {-2, -1, 0, 1, 2, 3, 4, 5, 7}, true));
    const auto next = manager.getStorage().currentID();
    for (const auto& descriptor : invalid) {
        EXPECT_THROW(manager.addFigure(descriptor), std::invalid_argument);
        EXPECT_EQ(manager.figureCount(), 0u);
        EXPECT_EQ(manager.getStorage().currentID(), next);
    }
}

TEST(DCMManagerCurveTest, InvalidControlAndExplicitIdsLeaveGeometryUnchanged) {
    DCMManager manager;
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    auto descriptor = FigureDescriptor::curve(1, std::vector<ID>{a, ID(999)}, {1, 1}, {0, 0, 1, 1});
    EXPECT_THROW(manager.addFigure(descriptor), std::runtime_error);
    EXPECT_EQ(manager.figureCount(), 1u);
    auto owned = bezier(); owned.id = ID(100);
    EXPECT_THROW(manager.addFigure(owned), std::invalid_argument);
    descriptor.pointIds = {a, a}; descriptor.id = a;
    EXPECT_THROW(manager.addFigure(descriptor), std::invalid_argument);
    EXPECT_EQ(manager.figureCount(), 1u);
}

TEST(DCMManagerCurveTest, InvalidUpdateBatchDoesNotMoveAnyControl) {
    DCMManager manager;
    const auto id = manager.addFigure(bezier());
    const auto before = manager.getFigure(id)->coords;
    EXPECT_THROW(manager.updateFigures({
        FigureUpdateDescriptor::curve(id, {0, 0, 0, 4, 4, 4, 4, 0}),
        FigureUpdateDescriptor::curve(id, {0, 0})}), std::invalid_argument);
    EXPECT_EQ(manager.getFigure(id)->coords, before);
    EXPECT_THROW(manager.updateFigure(FigureUpdateDescriptor::curve(id,
        {0, 0, 0, 4, 4, 4, 4, std::numeric_limits<double>::infinity()})), std::invalid_argument);
    EXPECT_EQ(manager.getFigure(id)->coords, before);
}

TEST(DCMManagerCurveTest, ControlsRespectExistingConstraintsDuringDrag) {
    DCMManager manager;
    const auto id = manager.addFigure(bezier());
    const auto controls = manager.getFigure(id)->pointIds;
    manager.addRequirement(RequirementDescriptor::fixPoint(controls[0]));
    manager.addRequirement(RequirementDescriptor::pointPointDist(controls[0], controls[1], 2));
    ASSERT_TRUE(manager.solve());
    manager.setSolveMode(SolveMode::DRAG);
    EXPECT_TRUE(manager.updateFigure(FigureUpdateDescriptor::curve(id, {10, 10, 0, 4, 2, 2, 2, 0})));
    const auto descriptor = manager.getFigure(id);
    EXPECT_NEAR(descriptor->coords[0], 0, 1e-12);
    EXPECT_NEAR(descriptor->coords[1], 0, 1e-12);
    EXPECT_NEAR(std::hypot(descriptor->coords[2], descriptor->coords[3]), 2, 1e-6);
}
