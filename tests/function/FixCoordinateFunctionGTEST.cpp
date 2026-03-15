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
    auto [id, pt] = storage.createPoint(3.0, 4.0);
    auto funcs = RequirementFunctionFactory::createFixPoint(pt);

    ASSERT_EQ(funcs.size(), 2u);

    EQ(funcs[0]->evaluate(), 0.0);
    EQ(funcs[1]->evaluate(), 0.0);

    EXPECT_EQ(funcs[0]->getVars()[0], pt->ptrX());
    EXPECT_EQ(funcs[1]->getVars()[0], pt->ptrY());
}

TEST(FixCoordinateFactoryTest, CreateFixPointDetectsMovement) {
    GeometryStorage storage;
    auto [id, pt] = storage.createPoint(3.0, 4.0);
    auto funcs = RequirementFunctionFactory::createFixPoint(pt);

    pt->x() = 10.0;
    pt->y() = 20.0;
    EQ(funcs[0]->evaluate(), 7.0);
    EQ(funcs[1]->evaluate(), 16.0);
}

TEST(FixCoordinateFactoryTest, CreateFixLine) {
    GeometryStorage storage;
    auto [id1, pt1] = storage.createPoint(1.0, 2.0);
    auto [id2, pt2] = storage.createPoint(5.0, 6.0);
    auto [lid, line] = storage.createLine(pt1, pt2);

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
    auto [id1, pt1] = storage.createPoint(1.0, 2.0);
    auto [id2, pt2] = storage.createPoint(5.0, 6.0);
    auto [lid, line] = storage.createLine(pt1, pt2);

    auto funcs = RequirementFunctionFactory::createFixLine(line);

    pt1->x() = 0.0;
    pt2->y() = 0.0;
    EQ(funcs[0]->evaluate(), -1.0);
    EQ(funcs[3]->evaluate(), -6.0);
}

TEST(FixCoordinateFactoryTest, CreateFixCircle) {
    GeometryStorage storage;
    auto [cid, center] = storage.createPoint(3.0, 4.0);
    auto [circId, circle] = storage.createCircle(center, 7.5);

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
    auto [cid, center] = storage.createPoint(3.0, 4.0);
    auto [circId, circle] = storage.createCircle(center, 7.5);

    auto funcs = RequirementFunctionFactory::createFixCircle(circle);

    center->x() = 0.0;
    center->y() = 0.0;
    circle->radius = 10.0;

    EQ(funcs[0]->evaluate(), -3.0);
    EQ(funcs[1]->evaluate(), -4.0);
    EQ(funcs[2]->evaluate(), 2.5);
}
