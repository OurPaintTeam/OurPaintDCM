#include <gtest/gtest.h>
#include "functions/RequirementFunction.h"
#include "RequirementFunctionFactory.h"
#include "GeometryStorage.h"
#include <cmath>

using namespace OurPaintDCM::Function;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

#define EQ(a, b) EXPECT_NEAR((a), (b), 1e-9)

TEST(FixCoordinateFunctionTest, EvaluateAtTarget) {
    double x = 5.0;
    std::vector<VAR> vars = {&x};
    FixCoordinateFunction f(RequirementType::ET_FIXPOINT, vars, 5.0);
    EQ(f.evaluate(), 0.0);
}

TEST(FixCoordinateFunctionTest, EvaluateDeviated) {
    double x = 7.0;
    std::vector<VAR> vars = {&x};
    FixCoordinateFunction f(RequirementType::ET_FIXPOINT, vars, 5.0);
    EQ(f.evaluate(), 2.0);
}

TEST(FixCoordinateFunctionTest, EvaluateNegativeDeviation) {
    double x = 3.0;
    std::vector<VAR> vars = {&x};
    FixCoordinateFunction f(RequirementType::ET_FIXPOINT, vars, 5.0);
    EQ(f.evaluate(), -2.0);
}

TEST(FixCoordinateFunctionTest, GradientIsOne) {
    double x = 5.0;
    std::vector<VAR> vars = {&x};
    FixCoordinateFunction f(RequirementType::ET_FIXPOINT, vars, 5.0);

    auto grad = f.gradient();
    EXPECT_EQ(grad.size(), 1u);
    EQ(grad[&x], 1.0);
}

TEST(FixCoordinateFunctionTest, VarCount) {
    double x = 0.0;
    std::vector<VAR> vars = {&x};
    FixCoordinateFunction f(RequirementType::ET_FIXPOINT, vars, 0.0);
    EXPECT_EQ(f.getVarCount(), 1u);
}

TEST(FixCoordinateFunctionTest, InvalidVarCountThrows) {
    double x = 0.0, y = 0.0;
    std::vector<VAR> vars = {&x, &y};
    EXPECT_THROW(
        FixCoordinateFunction(RequirementType::ET_FIXPOINT, vars, 0.0),
        std::invalid_argument
    );
}

TEST(FixCoordinateFunctionTest, TypePreserved) {
    double x = 0.0;
    std::vector<VAR> vars = {&x};

    FixCoordinateFunction fp(RequirementType::ET_FIXPOINT, vars, 0.0);
    EXPECT_EQ(fp.getType(), RequirementType::ET_FIXPOINT);

    FixCoordinateFunction fl(RequirementType::ET_FIXLINE, vars, 0.0);
    EXPECT_EQ(fl.getType(), RequirementType::ET_FIXLINE);

    FixCoordinateFunction fc(RequirementType::ET_FIXCIRCLE, vars, 0.0);
    EXPECT_EQ(fc.getType(), RequirementType::ET_FIXCIRCLE);
}

TEST(FixCoordinateFunctionTest, TracksVariableChanges) {
    double x = 5.0;
    std::vector<VAR> vars = {&x};
    FixCoordinateFunction f(RequirementType::ET_FIXPOINT, vars, 5.0);
    EQ(f.evaluate(), 0.0);

    x = 10.0;
    EQ(f.evaluate(), 5.0);
}

TEST(FixCoordinateFactoryTest, CreateFixPoint) {
    GeometryStorage storage;
    const ID id = storage.createPoint(3.0, 4.0);
    Point2D* pt = storage.get<Point2D>(id);
    ASSERT_NE(pt, nullptr);
    auto funcs = RequirementFunctionFactory::createFixPoint(pt);

    ASSERT_EQ(funcs.size(), 2u);

    EQ(funcs[0]->evaluate(), 0.0);
    EQ(funcs[1]->evaluate(), 0.0);

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
    EQ(funcs[0]->evaluate(), 7.0);
    EQ(funcs[1]->evaluate(), 16.0);
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
        EQ(f->evaluate(), 0.0);
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
    EQ(funcs[0]->evaluate(), -1.0);
    EQ(funcs[3]->evaluate(), -6.0);
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
        EQ(f->evaluate(), 0.0);
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

    EQ(funcs[0]->evaluate(), -3.0);
    EQ(funcs[1]->evaluate(), -4.0);
    EQ(funcs[2]->evaluate(), 2.5);
}
