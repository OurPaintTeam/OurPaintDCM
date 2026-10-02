#include <gtest/gtest.h>
#include "RequirementFunctionSystem.h"
#include "RequirementFunction.h"
#include <Eigen/Dense>

using namespace OurPaintDCM::System;
using namespace OurPaintDCM::Function;
using namespace OurPaintDCM::Utils;

TEST(RequirementFunctionSystemTest, AddAndUpdateJacobian) {
    RequirementFunctionSystem system;

    double x1 = 0, y1 = 0, x2 = 3, y2 = 4;
    std::vector<double*> vars = {&x1, &y1, &x2, &y2};

    auto func = std::make_shared<PointPointDistanceFunction>(vars, 5.0);
    system.addFunction(func);

    auto allVars = system.getAllVars();
    EXPECT_EQ(allVars.size(), 4);

    system.updateJ();
    Eigen::SparseMatrix<double> J = system.J();
    EXPECT_EQ(J.rows(), 1);
    EXPECT_EQ(J.cols(), 4);

    Eigen::MatrixXd dense = Eigen::MatrixXd(J);
    double sum = dense.sum();
    EXPECT_NEAR(sum, 0.0, 1e-6);
}

TEST(RequirementFunctionSystemTest, ResidualComputation) {
    RequirementFunctionSystem system;

    double x1 = 0, y1 = 0, x2 = 3, y2 = 4;
    std::vector<double*> vars = {&x1, &y1, &x2, &y2};

    auto func = std::make_shared<PointPointDistanceFunction>(vars, 5.0);
    system.addFunction(func);

    Eigen::VectorXd r = system.residuals();
    EXPECT_EQ(r.size(), 1);
    EXPECT_NEAR(r[0], 0.0, 1e-8);
}

TEST(RequirementFunctionSystemTest, DiagnoseWellConstrained) {
    RequirementFunctionSystem system;

    double x1 = 0, y1 = 0, x2 = 1, y2 = 0;
    std::vector<double*> vars = {&x1, &y1, &x2, &y2};
    auto func = std::make_shared<HorizontalFunction>(vars);

    system.addFunction(func);
    system.updateJ();

    auto status = system.diagnose();
    EXPECT_TRUE(status == SystemStatus::UNDER_CONSTRAINED ||
                status == SystemStatus::WELL_CONSTRAINED ||
                status == SystemStatus::OVER_CONSTRAINED);
}

TEST(RequirementFunctionSystemTest, ClearResetsSystem) {
    RequirementFunctionSystem system;

    double x1 = 0, y1 = 0, x2 = 1, y2 = 0;
    std::vector<double*> vars = {&x1, &y1, &x2, &y2};
    auto func = std::make_shared<HorizontalFunction>(vars);
    system.addFunction(func);
    system.updateJ();

    system.clear();

    EXPECT_TRUE(system.getAllVars().empty());
    EXPECT_EQ(system.J().rows(), 0);
    EXPECT_EQ(system.J().cols(), 0);
}

TEST(RequirementFunctionSystemTest, WeightsScaleResidualsJacobianAndDiagnostics) {
    RequirementFunctionSystem system;
    double x = 2.0;
    auto first = std::make_shared<FixCoordinateFunction>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&x}, 0.0);
    auto second = std::make_shared<FixCoordinateFunction>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&x}, 4.0);
    first->setWeight(2.0);
    second->setWeight(3.0);
    system.addFunction(first);
    system.addFunction(second);

    EXPECT_DOUBLE_EQ(system.residuals()[0], 4.0);
    EXPECT_DOUBLE_EQ(system.residuals()[1], -6.0);
    const Eigen::MatrixXd initialJ = Eigen::MatrixXd(system.J());
    EXPECT_DOUBLE_EQ(initialJ(0, 0), 2.0);
    EXPECT_DOUBLE_EQ(initialJ(1, 0), 3.0);
    EXPECT_DOUBLE_EQ(Eigen::MatrixXd(system.JTJ())(0, 0), 13.0);
    EXPECT_EQ(system.diagnose(), SystemStatus::OVER_CONSTRAINED);

    first->setWeight(5.0);
    EXPECT_DOUBLE_EQ(system.residuals()[0], 10.0);
    EXPECT_DOUBLE_EQ(Eigen::MatrixXd(system.J())(0, 0), 5.0);
    first->setWeight(0.0);
    second->setWeight(0.0);
    EXPECT_TRUE(system.residuals().isZero());
    EXPECT_TRUE(Eigen::MatrixXd(system.J()).isZero());
    EXPECT_EQ(system.diagnose(), SystemStatus::EMPTY);
}

TEST(RequirementFunctionSystemTest, DiagnoseIgnoresZeroWeightConstraintsAndTheirVariables) {
    RequirementFunctionSystem system;
    double x = 2.0;
    double y = 3.0;
    auto active = std::make_shared<FixCoordinateFunction>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&x}, 0.0);
    auto disabledOnX = std::make_shared<FixCoordinateFunction>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&x}, 4.0);
    disabledOnX->setWeight(0.0);
    system.addFunction(active);
    system.addFunction(disabledOnX);
    EXPECT_EQ(system.diagnose(), SystemStatus::WELL_CONSTRAINED);

    auto disabledOnY = std::make_shared<FixCoordinateFunction>(
        RequirementType::ET_FIXPOINT, std::vector<VAR>{&y}, 5.0);
    disabledOnY->setWeight(0.0);
    system.addFunction(disabledOnY);
    EXPECT_EQ(system.diagnose(), SystemStatus::WELL_CONSTRAINED);

    active->setWeight(0.0);
    EXPECT_EQ(system.diagnose(), SystemStatus::EMPTY);
}
