#include "GeometryStorage.h"
#include <gtest/gtest.h>
#include <limits>
#include <numbers>

using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {
ID makeCurve(GeometryStorage& storage, unsigned degree,
             std::vector<FigureData::PointData> points, std::vector<double> weights,
             std::vector<double> knots, bool periodic = false) {
    FigureData data;
    data.points = std::move(points);
    data.degree = degree;
    data.weights = std::move(weights);
    data.knots = std::move(knots);
    data.periodic = periodic;
    return storage.createFigure(FigureType::ET_CURVE2D, data);
}

void expectPoint(const Point2D& point, double x, double y) {
    EXPECT_NEAR(point.x(), x, 1e-12);
    EXPECT_NEAR(point.y(), y, 1e-12);
}
}

TEST(Curve2DTest, CubicBezierMatchesBernsteinPolynomial) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 3, {{0, 0}, {0, 2}, {2, 2}, {2, 0}},
                              {1, 1, 1, 1}, {0, 0, 0, 0, 1, 1, 1, 1});
    for (double u : {0.0, 0.1, 0.5, 0.75, 1.0}) {
        expectPoint(storage.evaluateCurve(id, u), 6 * u * u - 4 * u * u * u, 6 * u * (1 - u));
    }
    const auto* curve = storage.get<Curve2D>(id);
    ASSERT_NE(curve, nullptr);
    EXPECT_DOUBLE_EQ(curve->firstParameter(), 0);
    EXPECT_DOUBLE_EQ(curve->lastParameter(), 1);
}

TEST(Curve2DTest, RationalQuadraticRepresentsQuarterCircle) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 2, {{1, 0}, {1, 1}, {0, 1}},
                              {1, std::numbers::sqrt2 / 2, 1}, {0, 0, 0, 1, 1, 1});
    expectPoint(storage.evaluateCurve(id, 0.5), std::numbers::sqrt2 / 2, std::numbers::sqrt2 / 2);
    for (double u : {0.0, 0.1, 0.5, 0.9, 1.0}) {
        const auto p = storage.evaluateCurve(id, u);
        EXPECT_NEAR(p.x() * p.x() + p.y() * p.y(), 1, 1e-12);
    }
}

TEST(Curve2DTest, NonUniformPolylineUsesItsKnotIntervals) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 1, {{0, 0}, {2, 4}, {8, 4}},
                              {1, 1, 1}, {0, 0, 2, 5, 5});
    expectPoint(storage.evaluateCurve(id, 1), 1, 2);
    expectPoint(storage.evaluateCurve(id, 2), 2, 4);
    expectPoint(storage.evaluateCurve(id, 3.5), 5, 4);
    expectPoint(storage.evaluateCurve(id, 5), 8, 4);
}

TEST(Curve2DTest, RepeatedInteriorKnotsJoinBezierSpans) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 2, {{0, 0}, {0, 2}, {2, 2}, {4, 2}, {4, 0}},
                              {1, 1, 1, 1, 1}, {0, 0, 0, 1, 1, 2, 2, 2});
    expectPoint(storage.evaluateCurve(id, 0.5), 0.5, 1.5);
    expectPoint(storage.evaluateCurve(id, 1), 2, 2);
    expectPoint(storage.evaluateCurve(id, 1.5), 3.5, 1.5);
}

TEST(Curve2DTest, UnclampedCurveEvaluatesBothDomainBoundaries) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 2, {{0, 0}, {2, 2}, {4, 0}, {8, 0}},
                              {1, 1, 1, 1}, {0, 0, 1, 2, 2, 3, 3});
    expectPoint(storage.evaluateCurve(id, 1), 1, 1);
    expectPoint(storage.evaluateCurve(id, 2), 4, 0);
    EXPECT_THROW((void)storage.evaluateCurve(id, 0), std::out_of_range);
    EXPECT_THROW((void)storage.evaluateCurve(id, 3), std::out_of_range);
}

TEST(Curve2DTest, PeriodicCurveWrapsAndClosesAtTheSeam) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 2, {{0, 0}, {2, 0}, {2, 2}, {0, 2}},
                              {1, 1, 1, 1}, {-2, -1, 0, 1, 2, 3, 4, 5, 6}, true);
    expectPoint(storage.evaluateCurve(id, 0), 1, 0);
    expectPoint(storage.evaluateCurve(id, 4), 1, 0);
    for (double u : {-3.5, 0.5, 4.5, 400.5}) expectPoint(storage.evaluateCurve(id, u), 1.75, 0.25);
    const auto left = storage.evaluateCurve(id, 4 - 1e-7);
    const auto right = storage.evaluateCurve(id, 1e-7);
    EXPECT_NEAR(left.x(), right.x(), 1e-6);
    EXPECT_NEAR(left.y(), right.y(), 1e-6);
}

TEST(Curve2DTest, PeriodicRepeatedKnotsAndNonzeroOrigin) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 2, {{0, 0}, {2, 0}, {2, 2}, {0, 2}},
                              {1, 2, 1, 2}, {-3, -3, -2, -2, -1, -1, 0, 0, 1}, true);
    expectPoint(storage.evaluateCurve(id, -2), 2, 0);
    expectPoint(storage.evaluateCurve(id, 0), 2, 0);
    const auto expected = storage.evaluateCurve(id, -1.5);
    for (double u : {-5.5, 0.5, 10.5}) {
        const auto p = storage.evaluateCurve(id, u);
        expectPoint(p, expected.x(), expected.y());
    }
}

TEST(Curve2DTest, WeightScalingPreservesTheCurve) {
    GeometryStorage storage;
    for (double scale : {1e-250, 1.0, 1e250}) {
        const auto id = makeCurve(storage, 1, {{0, 0}, {6, 3}}, {scale, 2 * scale}, {0, 0, 1, 1});
        expectPoint(storage.evaluateCurve(id, 0.5), 4, 2);
        expectPoint(storage.evaluateCurve(id, 0), 0, 0);
        expectPoint(storage.evaluateCurve(id, 1), 6, 3);
    }
}

TEST(Curve2DTest, InvalidParametersAndMissingCurveAreRejected) {
    GeometryStorage storage;
    const auto id = makeCurve(storage, 1, {{0, 0}, {2, 0}}, {1, 1}, {0, 0, 1, 1});
    EXPECT_THROW((void)storage.evaluateCurve(id, -0.1), std::out_of_range);
    EXPECT_THROW((void)storage.evaluateCurve(id, 1.1), std::out_of_range);
    EXPECT_THROW((void)storage.evaluateCurve(id, std::numeric_limits<double>::infinity()), std::invalid_argument);
    EXPECT_THROW((void)storage.evaluateCurve(id, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW((void)storage.evaluateCurve(ID(999), 0.5), std::runtime_error);
    EXPECT_THROW((void)storage.evaluateCurve(storage.get<Curve2D>(id)->pointIds[0], 0.5), std::runtime_error);
}

TEST(Curve2DTest, StorageTracksControlsCachesAndSlotReuse) {
    GeometryStorage storage;
    const auto a = storage.createPoint(0, 0), b = storage.createPoint(2, 0);
    const auto id = *storage.createCurve(2, {a, b, a}, {1, 1, 1}, {0, 0, 0, 1, 1, 1});
    EXPECT_EQ(storage.getDependencies(id), (std::vector<ID>{a, b, a}));
    EXPECT_EQ(storage.getDependents(a), (std::vector<ID>{id}));
    EXPECT_EQ(storage.getIDsByType(FigureType::ET_CURVE2D), (std::vector<ID>{id}));
    EXPECT_EQ(storage.curveCount(), 1u);
    EXPECT_EQ(storage.size(), 3u);
    EXPECT_EQ(storage.remove(a), RemoveResult::BlockedByDependents);
    const auto slot = storage.getEntry(id)->slot;
    const auto* ptr = storage.get<Curve2D>(id);
    for (int i = 0; i < 50; ++i)
        ASSERT_TRUE(storage.createCurve(1, {a, b}, {1, 1}, {0, 0, 1, 1}));
    EXPECT_EQ(storage.get<Curve2D>(id), ptr);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
    EXPECT_EQ(storage.remove(id), RemoveResult::Ok);
    const auto next = *storage.createCurve(1, {a, b}, {1, 1}, {0, 0, 1, 1});
    EXPECT_EQ(storage.getEntry(next)->slot, slot);
    EXPECT_EQ(storage.curvesWithIds().size(), 51u);
    EXPECT_TRUE(storage.buildObjectGraph().hasVertex(next));
    EXPECT_TRUE(storage.buildObjectSubgraph(a)->hasVertex(next));
    EXPECT_EQ(storage.remove(a, true), RemoveResult::Ok);
    EXPECT_EQ(storage.curveCount(), 0u);
    EXPECT_EQ(storage.pointCount(), 1u);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
    storage.clear();
    EXPECT_TRUE(storage.curvesWithIds().empty());
    EXPECT_TRUE(storage.empty());
}

TEST(Curve2DTest, InvalidControlIdsDoNotConsumeIdsOrChangeStorage) {
    GeometryStorage storage;
    const auto a = storage.createPoint(0, 0);
    const auto next = storage.currentID();
    EXPECT_FALSE(storage.createCurve(1, {a, ID(999)}, {1, 1}, {0, 0, 1, 1}));
    EXPECT_EQ(storage.currentID(), next);
    EXPECT_EQ(storage.size(), 1u);
}
