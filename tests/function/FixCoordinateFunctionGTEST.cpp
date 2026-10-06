#include <gtest/gtest.h>
#include "functions/RequirementFunction.h"
#include "RequirementFunctionFactory.h"
#include "GeometryStorage.h"
#include <cmath>

using namespace OurPaintDCM::Function;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

#define EQ(a, b) EXPECT_NEAR((a), (b), 1e-9)

TEST(RequirementBindingTest, PreservesFixType) {
    double x=0;
    for (const auto type : {RequirementType::ET_FIXPOINT,RequirementType::ET_FIXLINE,RequirementType::ET_FIXCIRCLE}) {
        const auto binding=RequirementFunctionFactory::bind(type,Math::ConstraintKind::FixCoordinate,{&x},0);
        EXPECT_EQ(binding->getType(),type);
    }
}

TEST(FixCoordinateFactoryTest, CreateFixPoint) {
    GeometryStorage storage;
    const ID id = storage.createPoint(3.0, 4.0);
    Point2D* pt = storage.get<Point2D>(id);
    ASSERT_NE(pt, nullptr);
    auto funcs = RequirementFunctionFactory::createFixPoint(pt);

    ASSERT_EQ(funcs.size(), 2u);

    EQ(funcs[0]->mathematical()->evaluate(), 0.0);
    EQ(funcs[1]->mathematical()->evaluate(), 0.0);

    EXPECT_EQ(funcs[0]->getVars()[0], pt->ptrX());
    EXPECT_EQ(funcs[1]->getVars()[0], pt->ptrY());
}

TEST(FixCoordinateFactoryTest, CreateFixPointDetectsMovement) {
    GeometryStorage storage;
    const ID id = storage.createPoint(3.0, 4.0);
    Point2D* pt = storage.get<Point2D>(id);
    ASSERT_NE(pt, nullptr);
    auto funcs = RequirementFunctionFactory::createFixPoint(pt);

    pt->x() = 10.0;
    pt->y() = 20.0;
    EQ(funcs[0]->mathematical()->evaluate(), 7.0);
    EQ(funcs[1]->mathematical()->evaluate(), 16.0);
}

TEST(FixCoordinateFactoryTest, CreateFixLine) {
    GeometryStorage storage;
    const ID id1 = storage.createPoint(1.0, 2.0);
    const ID id2 = storage.createPoint(5.0, 6.0);
    Point2D* pt1 = storage.get<Point2D>(id1);
    Point2D* pt2 = storage.get<Point2D>(id2);
    auto lineOpt = storage.createLine(id1, id2);
    ASSERT_TRUE(lineOpt.has_value());
    Line2D* line = storage.get<Line2D>(*lineOpt);
    ASSERT_NE(line, nullptr);

    auto funcs = RequirementFunctionFactory::createFixLine(line);
    ASSERT_EQ(funcs.size(), 4u);

    for (auto& f : funcs) {
        EQ(f->mathematical()->evaluate(), 0.0);
    }

    EXPECT_EQ(funcs[0]->getVars()[0], pt1->ptrX());
    EXPECT_EQ(funcs[1]->getVars()[0], pt1->ptrY());
    EXPECT_EQ(funcs[2]->getVars()[0], pt2->ptrX());
    EXPECT_EQ(funcs[3]->getVars()[0], pt2->ptrY());
}

TEST(FixCoordinateFactoryTest, CreateFixLineDetectsMovement) {
    GeometryStorage storage;
    const ID id1 = storage.createPoint(1.0, 2.0);
    const ID id2 = storage.createPoint(5.0, 6.0);
    Point2D* pt1 = storage.get<Point2D>(id1);
    Point2D* pt2 = storage.get<Point2D>(id2);
    auto lineOpt = storage.createLine(id1, id2);
    ASSERT_TRUE(lineOpt.has_value());
    Line2D* line = storage.get<Line2D>(*lineOpt);
    ASSERT_NE(line, nullptr);

    auto funcs = RequirementFunctionFactory::createFixLine(line);

    pt1->x() = 0.0;
    pt2->y() = 0.0;
    EQ(funcs[0]->mathematical()->evaluate(), -1.0);
    EQ(funcs[3]->mathematical()->evaluate(), -6.0);
}

TEST(FixCoordinateFactoryTest, CreateFixCircle) {
    GeometryStorage storage;
    const ID cid = storage.createPoint(3.0, 4.0);
    Point2D* center = storage.get<Point2D>(cid);
    ASSERT_NE(center, nullptr);
    auto circOpt = storage.createCircle(cid, 7.5);
    ASSERT_TRUE(circOpt.has_value());
    Circle2D* circle = storage.get<Circle2D>(*circOpt);
    ASSERT_NE(circle, nullptr);

    auto funcs = RequirementFunctionFactory::createFixCircle(circle);
    ASSERT_EQ(funcs.size(), 3u);

    for (auto& f : funcs) {
        EQ(f->mathematical()->evaluate(), 0.0);
    }

    EXPECT_EQ(funcs[0]->getVars()[0], center->ptrX());
    EXPECT_EQ(funcs[1]->getVars()[0], center->ptrY());
    EXPECT_EQ(funcs[2]->getVars()[0], circle->ptrRadius());
}

TEST(FixCoordinateFactoryTest, CreateFixCircleDetectsMovement) {
    GeometryStorage storage;
    const ID cid = storage.createPoint(3.0, 4.0);
    Point2D* center = storage.get<Point2D>(cid);
    ASSERT_NE(center, nullptr);
    auto circOpt = storage.createCircle(cid, 7.5);
    ASSERT_TRUE(circOpt.has_value());
    Circle2D* circle = storage.get<Circle2D>(*circOpt);
    ASSERT_NE(circle, nullptr);

    auto funcs = RequirementFunctionFactory::createFixCircle(circle);

    center->x() = 0.0;
    center->y() = 0.0;
    circle->radius = 10.0;

    EQ(funcs[0]->mathematical()->evaluate(), -3.0);
    EQ(funcs[1]->mathematical()->evaluate(), -4.0);
    EQ(funcs[2]->mathematical()->evaluate(), 2.5);
}
