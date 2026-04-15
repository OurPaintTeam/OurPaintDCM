#include <gtest/gtest.h>
#include <algorithm>
#include <stdexcept>

#include "GeometryStorage.h"

using namespace OurPaintDCM::Figures;

namespace {
bool containsId(const std::vector<ID>& ids, ID id) {
    return std::find(ids.begin(), ids.end(), id) != ids.end();
}
} // namespace

TEST(GeometryStorageTest, CreatePointAndAccess) {
    GeometryStorage storage;

    const ID id = storage.createPoint(1.5, -2.0);
    Point2D* p = storage.get<Point2D>(id);
    ASSERT_NE(p, nullptr);

    EXPECT_TRUE(storage.contains(id));
    EXPECT_EQ(storage.getType(id).value(), FigureType::ET_POINT2D);
    EXPECT_EQ(storage.getEntry(id)->type, FigureType::ET_POINT2D);

    auto point = storage.get<Point2D>(id);
    ASSERT_NE(point, nullptr);
    EXPECT_DOUBLE_EQ(point->x(), 1.5);
    EXPECT_DOUBLE_EQ(point->y(), -2.0);

    EXPECT_EQ(storage.size(), 1u);
    EXPECT_EQ(storage.pointCount(), 1u);
    EXPECT_TRUE(storage.getIDsByType(FigureType::ET_POINT2D).size() == 1u);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageTest, CreateLineCircleArcAndDependencies) {
    GeometryStorage storage;

    const ID p1Id = storage.createPoint(0.0, 0.0);
    const ID p2Id = storage.createPoint(3.0, 0.0);
    const ID cId  = storage.createPoint(1.0, 1.0);

    auto lineOpt = storage.createLine(p1Id, p2Id);
    auto circleOpt = storage.createCircle(cId, 5.0);
    auto arcOpt = storage.createArc(p1Id, p2Id, cId);
    ASSERT_TRUE(lineOpt && circleOpt && arcOpt);
    const ID lineId = *lineOpt;
    const ID circleId = *circleOpt;
    const ID arcId = *arcOpt;

    auto lineDeps = storage.getDependencies(lineId);
    EXPECT_EQ(lineDeps.size(), 2u);
    EXPECT_TRUE(containsId(lineDeps, p1Id));
    EXPECT_TRUE(containsId(lineDeps, p2Id));

    auto circleDeps = storage.getDependencies(circleId);
    EXPECT_EQ(circleDeps.size(), 1u);
    EXPECT_TRUE(containsId(circleDeps, cId));

    auto arcDeps = storage.getDependencies(arcId);
    EXPECT_EQ(arcDeps.size(), 3u);
    EXPECT_TRUE(containsId(arcDeps, p1Id));
    EXPECT_TRUE(containsId(arcDeps, p2Id));
    EXPECT_TRUE(containsId(arcDeps, cId));

    auto p1Dependents = storage.getDependents(p1Id);
    EXPECT_EQ(p1Dependents.size(), 2u);
    EXPECT_TRUE(containsId(p1Dependents, lineId));
    EXPECT_TRUE(containsId(p1Dependents, arcId));

    auto cDependents = storage.getDependents(cId);
    EXPECT_EQ(cDependents.size(), 2u);
    EXPECT_TRUE(containsId(cDependents, circleId));
    EXPECT_TRUE(containsId(cDependents, arcId));
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageTest, CreateLineInvalidIds) {
    GeometryStorage storage;
    const ID a = storage.createPoint(0, 0);
    EXPECT_FALSE(storage.createLine(a, ID(999)).has_value());
    EXPECT_FALSE(storage.createLine(ID(999), a).has_value());
}

TEST(GeometryStorageTest, CreateFigureFromData) {
    GeometryStorage storage;

    FigureData pointData;
    pointData.points.push_back({1.0, 2.0});
    auto pointId = storage.createFigure(FigureType::ET_POINT2D, pointData);
    EXPECT_EQ(storage.getType(pointId).value(), FigureType::ET_POINT2D);

    FigureData lineData;
    lineData.points.push_back({0.0, 0.0});
    lineData.points.push_back({1.0, 0.0});
    auto lineId = storage.createFigure(FigureType::ET_LINE, lineData);
    EXPECT_EQ(storage.getType(lineId).value(), FigureType::ET_LINE);

    FigureData circleData;
    circleData.center = {5.0, 5.0};
    circleData.radius = 2.5;
    auto circleId = storage.createFigure(FigureType::ET_CIRCLE, circleData);
    EXPECT_EQ(storage.getType(circleId).value(), FigureType::ET_CIRCLE);

    FigureData arcData;
    arcData.points.push_back({0.0, 0.0});
    arcData.points.push_back({2.0, 0.0});
    arcData.center = {1.0, 1.0};
    auto arcId = storage.createFigure(FigureType::ET_ARC, arcData);
    EXPECT_EQ(storage.getType(arcId).value(), FigureType::ET_ARC);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageTest, CreateFigureInvalidData) {
    GeometryStorage storage;

    FigureData emptyPoints;
    EXPECT_THROW((void)storage.createFigure(FigureType::ET_POINT2D, emptyPoints), std::runtime_error);

    FigureData onePoint;
    onePoint.points.push_back({0.0, 0.0});
    EXPECT_THROW((void)storage.createFigure(FigureType::ET_LINE, onePoint), std::runtime_error);
    EXPECT_THROW((void)storage.createFigure(FigureType::ET_ARC, onePoint), std::runtime_error);
    EXPECT_THROW((void)storage.createFigure(static_cast<FigureType>(255), onePoint), std::runtime_error);
}

TEST(GeometryStorageTest, RemoveAndCascade) {
    GeometryStorage storage;

    const ID p1Id = storage.createPoint(0.0, 0.0);
    const ID p2Id = storage.createPoint(1.0, 0.0);
    auto lineOpt = storage.createLine(p1Id, p2Id);
    ASSERT_TRUE(lineOpt);
    const ID lineId = *lineOpt;

    EXPECT_EQ(storage.remove(p1Id, false), RemoveResult::BlockedByDependents);

    EXPECT_EQ(storage.remove(p1Id, true), RemoveResult::Ok);
    EXPECT_FALSE(storage.contains(p1Id));
    EXPECT_FALSE(storage.contains(lineId));
    EXPECT_TRUE(storage.contains(p2Id));
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageTest, RemoveNotFound) {
    GeometryStorage storage;
    EXPECT_EQ(storage.remove(ID(123)), RemoveResult::NotFound);
}

TEST(GeometryStorageTest, ClearAndStats) {
    GeometryStorage storage;

    EXPECT_TRUE(storage.empty());
    EXPECT_EQ(storage.currentID().id, 1ull);

    (void)storage.createPoint(0.0, 0.0);
    (void)storage.createPoint(1.0, 1.0);
    EXPECT_FALSE(storage.empty());
    EXPECT_EQ(storage.pointCount(), 2u);
    EXPECT_EQ(storage.currentID().id, 3ull);

    storage.clear();
    EXPECT_TRUE(storage.empty());
    EXPECT_EQ(storage.size(), 0u);
    EXPECT_EQ(storage.pointCount(), 0u);
    EXPECT_EQ(storage.lineCount(), 0u);
    EXPECT_EQ(storage.circleCount(), 0u);
    EXPECT_EQ(storage.arcCount(), 0u);
    EXPECT_EQ(storage.currentID().id, 1ull);
}

TEST(GeometryStorageTest, SpansAndCounts) {
    GeometryStorage storage;

    EXPECT_TRUE(storage.pointsWithIds().empty());
    EXPECT_TRUE(storage.linesWithIds().empty());
    EXPECT_TRUE(storage.circlesWithIds().empty());
    EXPECT_TRUE(storage.arcsWithIds().empty());

    const ID p1Id = storage.createPoint(0.0, 0.0);
    const ID p2Id = storage.createPoint(1.0, 0.0);
    const ID cId  = storage.createPoint(2.0, 2.0);
    ASSERT_TRUE(storage.createLine(p1Id, p2Id).has_value());
    ASSERT_TRUE(storage.createCircle(cId, 3.0).has_value());
    ASSERT_TRUE(storage.createArc(p1Id, p2Id, cId).has_value());

    EXPECT_EQ(storage.pointCount(), 3u);
    EXPECT_EQ(storage.lineCount(), 1u);
    EXPECT_EQ(storage.circleCount(), 1u);
    EXPECT_EQ(storage.arcCount(), 1u);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageTest, BuildObjectGraph) {
    GeometryStorage storage;

    const ID p1Id = storage.createPoint(0.0, 0.0);
    const ID p2Id = storage.createPoint(1.0, 0.0);
    const ID cId  = storage.createPoint(2.0, 2.0);

    auto lineOpt = storage.createLine(p1Id, p2Id);
    auto circleOpt = storage.createCircle(cId, 3.0);
    auto arcOpt = storage.createArc(p1Id, p2Id, cId);
    ASSERT_TRUE(lineOpt && circleOpt && arcOpt);
    const ID lineId = *lineOpt;
    const ID circleId = *circleOpt;
    const ID arcId = *arcOpt;

    auto graph = storage.buildObjectGraph();

    EXPECT_EQ(graph.vertexCount(), 6u);
    EXPECT_EQ(graph.edgeCount(), 6u);
    EXPECT_TRUE(graph.hasEdge(lineId, p1Id));
    EXPECT_TRUE(graph.hasEdge(lineId, p2Id));
    EXPECT_TRUE(graph.hasEdge(circleId, cId));
    EXPECT_TRUE(graph.hasEdge(arcId, p1Id));
    EXPECT_TRUE(graph.hasEdge(arcId, p2Id));
    EXPECT_TRUE(graph.hasEdge(arcId, cId));
}

TEST(GeometryStorageTest, BuildObjectSubgraph) {
    GeometryStorage storage;

    const ID p1Id = storage.createPoint(0.0, 0.0);
    const ID p2Id = storage.createPoint(1.0, 0.0);
    auto lineOpt = storage.createLine(p1Id, p2Id);
    ASSERT_TRUE(lineOpt);
    const ID lineId = *lineOpt;
    const ID p3Id = storage.createPoint(5.0, 5.0);

    auto subgraph = storage.buildObjectSubgraph(lineId);
    ASSERT_TRUE(subgraph.has_value());
    EXPECT_EQ(subgraph->vertexCount(), 3u);
    EXPECT_TRUE(subgraph->hasEdge(lineId, p1Id));
    EXPECT_TRUE(subgraph->hasEdge(lineId, p2Id));

    auto pointGraph = storage.buildObjectSubgraph(p1Id);
    ASSERT_TRUE(pointGraph.has_value());
    EXPECT_EQ(pointGraph->vertexCount(), 3u);
    EXPECT_TRUE(pointGraph->hasEdge(p1Id, lineId));
    EXPECT_TRUE(pointGraph->hasEdge(lineId, p2Id));

    auto isolated = storage.buildObjectSubgraph(p3Id);
    ASSERT_TRUE(isolated.has_value());
    EXPECT_EQ(isolated->vertexCount(), 1u);
    EXPECT_FALSE(isolated->hasEdge(p3Id, p3Id));

    EXPECT_FALSE(storage.buildObjectSubgraph(ID(999)).has_value());
}

TEST(GeometryStorageTest, SharedPointBetweenLines) {
    GeometryStorage storage;

    const ID pSharedId = storage.createPoint(0.0, 0.0);
    const ID p1Id = storage.createPoint(1.0, 0.0);
    const ID p2Id = storage.createPoint(2.0, 0.0);

    auto l1 = storage.createLine(pSharedId, p1Id);
    auto l2 = storage.createLine(pSharedId, p2Id);
    ASSERT_TRUE(l1 && l2);
    const ID line1Id = *l1;
    const ID line2Id = *l2;

    auto dependents = storage.getDependents(pSharedId);
    EXPECT_EQ(dependents.size(), 2u);
    EXPECT_TRUE(std::find(dependents.begin(), dependents.end(), line1Id) != dependents.end());
    EXPECT_TRUE(std::find(dependents.begin(), dependents.end(), line2Id) != dependents.end());

    auto subgraph = storage.buildObjectSubgraph(pSharedId);
    ASSERT_TRUE(subgraph.has_value());
    EXPECT_EQ(subgraph->vertexCount(), 5u);
    EXPECT_TRUE(subgraph->hasEdge(pSharedId, line1Id));
    EXPECT_TRUE(subgraph->hasEdge(pSharedId, line2Id));
    EXPECT_TRUE(subgraph->hasEdge(line1Id, p1Id));
    EXPECT_TRUE(subgraph->hasEdge(line2Id, p2Id));
}

TEST(GeometryStorageTest, GetTypeMismatch) {
    GeometryStorage storage;
    const ID id = storage.createPoint(0.0, 0.0);
    EXPECT_EQ(storage.get<Line2D>(id), nullptr);
}

TEST(GeometryStorageTest, GetNotFound) {
    GeometryStorage storage;
    EXPECT_EQ(storage.get<Point2D>(ID(999)), nullptr);
}

#ifndef NDEBUG
TEST(GeometryStorageTest, SlotReuse_NoUnboundedGrowth) {
    GeometryStorage storage;
    for (int k = 0; k < 5; ++k) {
        const ID p = storage.createPoint(1.0, 2.0);
        EXPECT_EQ(storage.remove(p, false), RemoveResult::Ok);
    }
    EXPECT_LE(storage.debugPointPoolSize(), 2u);
    EXPECT_TRUE(storage.validate());
}

TEST(GeometryStorageTest, ValidateAfterStress) {
    GeometryStorage storage;
    const ID a = storage.createPoint(0, 0);
    const ID b = storage.createPoint(1, 0);
    auto line = storage.createLine(a, b);
    ASSERT_TRUE(line);
    EXPECT_TRUE(storage.validate());
    EXPECT_EQ(storage.remove(a, true), RemoveResult::Ok);
    EXPECT_TRUE(storage.validate());
}
#endif
