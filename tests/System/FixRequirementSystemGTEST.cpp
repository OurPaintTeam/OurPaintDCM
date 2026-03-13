#include <gtest/gtest.h>
#include "RequirementSystem.h"
#include "GeometryStorage.h"

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

class FixRequirementSystemTest : public ::testing::Test {
protected:
    GeometryStorage storage;
    ID p1Id, p2Id, centerId;
    ID lineId;
    ID circleId;

    void SetUp() override {
        auto [id1, pt1] = storage.createPoint(1.0, 2.0);
        auto [id2, pt2] = storage.createPoint(5.0, 6.0);
        auto [id3, pt3] = storage.createPoint(3.0, 4.0);

        p1Id = id1;
        p2Id = id2;
        centerId = id3;

        auto [lid, l] = storage.createLine(pt1, pt2);
        lineId = lid;

        auto [cid, c] = storage.createCircle(pt3, 7.5);
        circleId = cid;
    }
};

TEST_F(FixRequirementSystemTest, AddFixPointVars) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);

    EXPECT_EQ(system.getAllVars().size(), 2u);
}

TEST_F(FixRequirementSystemTest, AddFixPointResiduals) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);

    auto res = system.residuals();
    EXPECT_EQ(res.size(), 2u);
    EXPECT_NEAR(res[0], 0.0, 1e-9);
    EXPECT_NEAR(res[1], 0.0, 1e-9);
}

TEST_F(FixRequirementSystemTest, AddFixPointDetectsDeviation) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);

    auto* pt = storage.get<Point2D>(p1Id);
    pt->x() = 100.0;
    pt->y() = 200.0;

    auto res = system.residuals();
    EXPECT_NEAR(res[0], 99.0, 1e-9);
    EXPECT_NEAR(res[1], 198.0, 1e-9);
}

TEST_F(FixRequirementSystemTest, AddFixPointJacobian) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);
    system.updateJ();

    auto J = system.J();
    EXPECT_EQ(J.rows(), 2);
    EXPECT_EQ(J.cols(), 2);
}

TEST_F(FixRequirementSystemTest, AddFixPointDiagnose) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);
    system.updateJ();

    auto status = system.diagnose();
    EXPECT_EQ(status, SystemStatus::WELL_CONSTRAINED);
}

TEST_F(FixRequirementSystemTest, AddFixLineVars) {
    RequirementSystem system(&storage);
    system.addFixLine(lineId);

    EXPECT_EQ(system.getAllVars().size(), 4u);
}

TEST_F(FixRequirementSystemTest, AddFixLineResiduals) {
    RequirementSystem system(&storage);
    system.addFixLine(lineId);

    auto res = system.residuals();
    EXPECT_EQ(res.size(), 4u);
    for (double r : res) {
        EXPECT_NEAR(r, 0.0, 1e-9);
    }
}

TEST_F(FixRequirementSystemTest, AddFixLineDetectsDeviation) {
    RequirementSystem system(&storage);
    system.addFixLine(lineId);

    auto* pt1 = storage.get<Point2D>(p1Id);
    pt1->x() = 0.0;

    auto res = system.residuals();
    EXPECT_NEAR(res[0], -1.0, 1e-9);
}

TEST_F(FixRequirementSystemTest, AddFixLineJacobian) {
    RequirementSystem system(&storage);
    system.addFixLine(lineId);
    system.updateJ();

    auto J = system.J();
    EXPECT_EQ(J.rows(), 4);
    EXPECT_EQ(J.cols(), 4);
}

TEST_F(FixRequirementSystemTest, AddFixLineDiagnose) {
    RequirementSystem system(&storage);
    system.addFixLine(lineId);
    system.updateJ();

    auto status = system.diagnose();
    EXPECT_EQ(status, SystemStatus::WELL_CONSTRAINED);
}

TEST_F(FixRequirementSystemTest, AddFixCircleVars) {
    RequirementSystem system(&storage);
    system.addFixCircle(circleId);

    EXPECT_EQ(system.getAllVars().size(), 3u);
}

TEST_F(FixRequirementSystemTest, AddFixCircleResiduals) {
    RequirementSystem system(&storage);
    system.addFixCircle(circleId);

    auto res = system.residuals();
    EXPECT_EQ(res.size(), 3u);
    for (double r : res) {
        EXPECT_NEAR(r, 0.0, 1e-9);
    }
}

TEST_F(FixRequirementSystemTest, AddFixCircleDetectsDeviation) {
    RequirementSystem system(&storage);
    system.addFixCircle(circleId);

    auto* c = storage.get<Circle2D>(circleId);
    c->center->x() = 0.0;
    c->radius = 20.0;

    auto res = system.residuals();
    EXPECT_NEAR(res[0], -3.0, 1e-9);
    EXPECT_NEAR(res[2], 12.5, 1e-9);
}

TEST_F(FixRequirementSystemTest, AddFixCircleJacobian) {
    RequirementSystem system(&storage);
    system.addFixCircle(circleId);
    system.updateJ();

    auto J = system.J();
    EXPECT_EQ(J.rows(), 3);
    EXPECT_EQ(J.cols(), 3);
}

TEST_F(FixRequirementSystemTest, AddFixCircleDiagnose) {
    RequirementSystem system(&storage);
    system.addFixCircle(circleId);
    system.updateJ();

    auto status = system.diagnose();
    EXPECT_EQ(status, SystemStatus::WELL_CONSTRAINED);
}

TEST_F(FixRequirementSystemTest, FixPointDependencyGraphNoExtraEdges) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);

    auto graph = system.buildDependencyGraph();
    EXPECT_TRUE(graph.hasVertex(p1Id));
    EXPECT_FALSE(graph.hasEdge(p1Id, p2Id));
}

TEST_F(FixRequirementSystemTest, FixLineDependencyGraphNoExtraEdges) {
    RequirementSystem system(&storage);
    system.addFixLine(lineId);

    auto graph = system.buildDependencyGraph();
    EXPECT_TRUE(graph.hasVertex(lineId));
    EXPECT_FALSE(graph.hasEdge(lineId, circleId));
}

TEST_F(FixRequirementSystemTest, FixCircleDependencyGraphNoExtraEdges) {
    RequirementSystem system(&storage);
    system.addFixCircle(circleId);

    auto graph = system.buildDependencyGraph();
    EXPECT_TRUE(graph.hasVertex(circleId));
    EXPECT_FALSE(graph.hasEdge(circleId, lineId));
}

TEST_F(FixRequirementSystemTest, FixWithOtherConstraints) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);
    system.addPointPointDist(p1Id, p2Id, 5.0);

    auto res = system.residuals();
    EXPECT_EQ(res.size(), 3u);
    EXPECT_NEAR(res[0], 0.0, 1e-9);
    EXPECT_NEAR(res[1], 0.0, 1e-9);
}

TEST_F(FixRequirementSystemTest, FixPointSharedVarsWithLine) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);
    system.addFixLine(lineId);

    auto vars = system.getAllVars();
    EXPECT_EQ(vars.size(), 4u);
}

TEST_F(FixRequirementSystemTest, InvalidIdThrows) {
    RequirementSystem system(&storage);

    EXPECT_THROW(system.addFixPoint(ID(999)), std::runtime_error);
    EXPECT_THROW(system.addFixLine(ID(999)), std::runtime_error);
    EXPECT_THROW(system.addFixCircle(ID(999)), std::runtime_error);
}

TEST_F(FixRequirementSystemTest, DescriptorInterface) {
    RequirementSystem system(&storage);

    auto desc = RequirementDescriptor::fixPoint(p1Id);
    auto reqId = system.addRequirement(desc);

    auto* entry = system.getRequirement(reqId);
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->type, RequirementType::ET_FIXPOINT);
    EXPECT_EQ(entry->objectIds.size(), 1u);
    EXPECT_EQ(entry->objectIds[0], p1Id);
}

TEST_F(FixRequirementSystemTest, DescriptorValidation) {
    RequirementDescriptor desc;
    desc.type = RequirementType::ET_FIXPOINT;
    desc.objectIds = {p1Id, p2Id};
    EXPECT_THROW(desc.validate(), std::invalid_argument);

    RequirementDescriptor desc2;
    desc2.type = RequirementType::ET_FIXLINE;
    desc2.objectIds = {};
    EXPECT_THROW(desc2.validate(), std::invalid_argument);
}

TEST_F(FixRequirementSystemTest, ClearRemovesFixConstraints) {
    RequirementSystem system(&storage);
    system.addFixPoint(p1Id);
    system.addFixCircle(circleId);
    system.updateJ();

    system.clear();

    EXPECT_TRUE(system.getAllVars().empty());
    EXPECT_EQ(system.J().rows(), 0);
    EXPECT_TRUE(system.getRequirements().empty());
}
