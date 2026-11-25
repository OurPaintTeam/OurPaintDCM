#include <gtest/gtest.h>
#include "SequentialConstraintSolve.h"
#include "ConstraintSolveOrder.h"
#include "RequirementSystem.h"
#include "sparse/SparseLevenbergMarquardtSolver.h"
#include <stdexcept>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {
class SequentialConstraintSolveTest : public ::testing::Test {
protected:
    GeometryStorage storage;
    ID a = storage.createPoint(0, 0);
    ID b = storage.createPoint(3, 0);
    ID c = storage.createPoint(8, 0);
    RequirementSystem system{&storage, std::vector<ID>{a, b, c}};
    ConstraintDecomposition prepare() {
        auto plan = buildConstraintDecomposition(system,
            decomposeByArticulationPoints(system.buildDependencyGraph()));
        EXPECT_TRUE(plan);
        return plan.value();
    }
    void chain() {
        system.addRequirement(RequirementDescriptor::fixPoint(a));
        system.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
        system.addRequirement(RequirementDescriptor::pointPointDist(b, c, 5));
    }
    static void optimize(TaskMatrix& task) {
        SparseLMSolver optimizer(100, 1e-3, 1e-10, 1e-10, 1e-16);
        optimizer.setTask(&task);
        optimizer.optimize();
    }
};

TEST_F(SequentialConstraintSolveTest, EachBlockSolvesOnceAndSharedCoordinatesStayExact) {
    chain();
    const auto plan = prepare();
    const auto order = chooseConstraintSolveOrder(system, plan);
    unsigned calls = 0;
    double sharedX = 0, sharedY = 0;
    const bool solved = solveSequentialConstraints(system, plan, order, {}, SolverType::LEVENBERG_MARQUARDT,
        1e-6, [&](TaskMatrix& task) {
            EXPECT_EQ(task.getValues().size(), 2U);
            optimize(task);
            if (calls++ == 0) {
                sharedX = storage.get<Point2D>(b)->x();
                sharedY = storage.get<Point2D>(b)->y();
            } else {
                EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->x(), sharedX);
                EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->y(), sharedY);
            }
        });
    EXPECT_TRUE(solved);
    EXPECT_EQ(calls, 2U);
    EXPECT_NEAR(storage.get<Point2D>(b)->x(), 5, 1e-6);
    EXPECT_NEAR(storage.get<Point2D>(c)->x(), 10, 1e-6);
}

TEST_F(SequentialConstraintSolveTest, UnsuccessfulSecondBlockRestoresSuccessfulFirstBlock) {
    chain();
    system.addRequirement(RequirementDescriptor::fixPoint(c));
    const auto plan = prepare();
    const std::vector<std::size_t> order{0, 1};
    EXPECT_FALSE(solveSequentialConstraints(system, plan, order, {}, SolverType::LEVENBERG_MARQUARDT, 1e-6, optimize));
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(a)->x(), 0);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->x(), 3);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(c)->x(), 8);
}

TEST_F(SequentialConstraintSolveTest, ExceptionAfterPartialSuccessRestoresAllParameters) {
    chain();
    const auto plan = prepare();
    const auto order = chooseConstraintSolveOrder(system, plan);
    unsigned calls = 0;
    EXPECT_THROW(solveSequentialConstraints(system, plan, order, {}, SolverType::LEVENBERG_MARQUARDT,
        1e-6, [&](TaskMatrix& task) {
            optimize(task);
            if (++calls == 2) throw std::runtime_error("test solver failure");
        }), std::runtime_error);
    EXPECT_EQ(calls, 2U);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->x(), 3);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(c)->x(), 8);
}

TEST_F(SequentialConstraintSolveTest, FrozenValuesAreNotReusedAcrossSolves) {
    chain();
    const auto plan = prepare();
    const auto order = chooseConstraintSolveOrder(system, plan);
    EXPECT_TRUE(solveSequentialConstraints(system, plan, order, {}, SolverType::LEVENBERG_MARQUARDT, 1e-6, optimize));
    storage.get<Point2D>(b)->x() = -3;
    storage.get<Point2D>(c)->x() = -8;
    EXPECT_TRUE(solveSequentialConstraints(system, plan, order, {}, SolverType::LEVENBERG_MARQUARDT, 1e-6, optimize));
    EXPECT_NEAR(storage.get<Point2D>(b)->x(), -5, 1e-6);
    EXPECT_NEAR(storage.get<Point2D>(c)->x(), -10, 1e-6);
}

TEST_F(SequentialConstraintSolveTest, FinalValidationRejectsDamageToAnEarlierBlock) {
    chain();
    const auto plan = prepare();
    const auto order = chooseConstraintSolveOrder(system, plan);
    unsigned calls = 0;
    EXPECT_FALSE(solveSequentialConstraints(system, plan, order, {}, SolverType::LEVENBERG_MARQUARDT,
        1e-6, [&](TaskMatrix& task) {
            if (++calls == 2) storage.get<Point2D>(a)->x() = 100;
            optimize(task);
        }));
    EXPECT_EQ(calls, 2U);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(a)->x(), 0);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(b)->x(), 3);
    EXPECT_DOUBLE_EQ(storage.get<Point2D>(c)->x(), 8);
}
} // namespace
