#include <gtest/gtest.h>
#include "functions/RequirementFunction.h"
#include <cmath>
#include <array>
#include <memory>
#include <unordered_map>
#include <vector>

using namespace OurPaintDCM::Function;

#define EQ(a, b) EXPECT_NEAR((a), (b), 1e-6)

namespace {

std::unordered_map<VAR, double> finiteDifferenceGradient(
    RequirementFunction& function,
    const std::vector<VAR>& vars,
    double eps = 1e-6) {
    std::unordered_map<VAR, double> grad;
    for (VAR var : vars) {
        const double original = *var;
        *var = original + eps;
        const double forward = function.evaluate();
        *var = original - eps;
        const double backward = function.evaluate();
        *var = original;
        grad[var] = (forward - backward) / (2.0 * eps);
    }
    return grad;
}

void expectGradientsNear(const std::unordered_map<VAR, double>& actual,
                         const std::unordered_map<VAR, double>& expected,
                         const std::vector<VAR>& vars,
                         double tolerance = 1e-5) {
    for (VAR var : vars) {
        const auto actualIt = actual.find(var);
        ASSERT_NE(actualIt, actual.end());
        const auto expectedIt = expected.find(var);
        ASSERT_NE(expectedIt, expected.end());
        EXPECT_NEAR(actualIt->second, expectedIt->second, tolerance);
    }
}

} // namespace

// ======== PointPointDistanceFunction ========
TEST(PointPointDistanceFunctionTest, EvaluateAndGradient) {
    double x1 = 0, y1 = 0, x2 = 3, y2 = 4;
    std::vector<VAR> vars = {&x1, &y1, &x2, &y2};

    PointPointDistanceFunction f(vars, 5.0);
    double val = f.evaluate();
    EQ(val, 0.0);

    auto grad = f.gradient();
    EQ(grad[&x1], -0.6);
    EQ(grad[&y1], -0.8);
    EQ(grad[&x2], 0.6);
    EQ(grad[&y2], 0.8);
}

// ======== PointOnPointFunction ========
TEST(PointOnPointFunctionTest, EvaluateAndGradient) {
    double x1 = 0, y1 = 0, x2 = 3, y2 = 4;
    std::vector<VAR> vars = {&x1, &y1, &x2, &y2};

    PointOnPointFunction f(vars);
    EQ(f.evaluate(), 5.0);
    auto grad = f.gradient();
    EQ(grad[&x1], -0.6);
    EQ(grad[&y1], -0.8);
    EQ(grad[&x2], 0.6);
    EQ(grad[&y2], 0.8);
}

// ======== PointLineDistanceFunction ========
TEST(PointLineDistanceFunctionTest, EvaluateSimple) {
    double px = 0, py = 1;
    double x1 = 1, y1 = 0, x2 = -1, y2 = 0;
    std::vector<VAR> vars = {&px, &py, &x1, &y1, &x2, &y2};

    PointLineDistanceFunction f(vars, 1);
    EQ(f.evaluate(), 0.0);
}

TEST(PointOnLineFunctionTest, EvaluateAndGradientNonAxisAligned) {
    double px = 2, py = 3;
    double x1 = 1, y1 = 1, x2 = 3, y2 = 5;
    std::vector<VAR> vars = {&px, &py, &x1, &y1, &x2, &y2};

    PointOnLineFunction f(vars);
    EQ(f.evaluate(), 0.0);
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars);
}

// ======== LineLineParallelFunction ========
TEST(LineCircleDistanceFunctionTest, UsesTargetAndClampsClosestPointToSegment) {
    double x1 = 4, y1 = 3, x2 = 10, y2 = 6, cx = 0, cy = 0, r = 2;
    const std::vector<VAR> vars = {&x1, &y1, &x2, &y2, &cx, &cy, &r};
    LineCircleDistanceFunction f(vars, 3.0);
    EQ(f.evaluate(), 0.0); // Closest point is the first endpoint, five units from center.
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars);

    cx = 14; cy = 9; // Closest point is the second endpoint.
    EQ(f.evaluate(), 0.0);
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars);

    cx = 5; cy = 7; // Interior projection on a non-axis-aligned segment.
    EQ(f.evaluate(), std::abs(6.0 * 4.0 - 3.0 * 1.0) / std::hypot(6.0, 3.0) - 5.0);
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars);
}

TEST(LineCircleDistanceFunctionTest, DegenerateSegmentAndCenterOnSegmentKeepRadiusDerivative) {
    double x1 = 4, y1 = 6, x2 = 4, y2 = 6, cx = 1, cy = 2, r = 2;
    const std::vector<VAR> vars = {&x1, &y1, &x2, &y2, &cx, &cy, &r};
    LineCircleDistanceFunction f(vars, 3.0);
    EQ(f.evaluate(), 0.0);
    auto gradient = f.gradient();
    EQ(gradient[&x1], 0.6);
    EQ(gradient[&y1], 0.8);
    EQ(gradient[&x2], 0.0);
    EQ(gradient[&y2], 0.0);
    EQ(gradient[&r], -1.0);

    x1 = 0; y1 = 2; x2 = 3; y2 = 2;
    EQ(f.evaluate(), -5.0);
    gradient = f.gradient();
    EQ(gradient[&r], -1.0);
    for (const auto& [variable, derivative] : gradient) {
        EXPECT_TRUE(std::isfinite(derivative));
    }
}

TEST(LineOnCircleFunctionTest, OppositeEndpointErrorsCannotCancel) {
    double x1 = 6, y1 = 7, x2 = 10, y2 = 7, cx = 3, cy = 7, r = 5;
    const std::vector<VAR> vars = {&x1, &y1, &x2, &y2, &cx, &cy, &r};
    LineOnCircleFunction f(vars);
    EQ(f.evaluate(), std::sqrt(8.0));
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars);
    x1 = -2; x2 = 8;
    EQ(f.evaluate(), 0.0);
}

TEST(LineCircleDistanceFunctionTest, SharedCenterCoordinatesAccumulateGradient) {
    double ax = 3, ay = 7, bx = 9, by = 11, radius = 2;
    const std::vector<VAR> vars = {&ax, &ay, &bx, &by, &ax, &ay, &radius};
    LineCircleDistanceFunction f(vars, 1.0);
    EQ(f.evaluate(), -3.0);
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars);
    LineOnCircleFunction onCircle(vars);
    expectGradientsNear(onCircle.gradient(), finiteDifferenceGradient(onCircle, vars), vars);
}

TEST(PointOnCircleFunctionTest, CenterRadiusAndSharedCoordinatesMatchFiniteDifferences) {
    double px = 6, py = 11, cx = 3, cy = 7, radius = 5;
    const std::vector<VAR> vars = {&px, &py, &cx, &cy, &radius};
    PointOnCircleFunction f(vars);
    EQ(f.evaluate(), 0.0);
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars);
    radius = 4;
    EQ(f.evaluate(), 1.0);
    const std::vector<VAR> sharedVars = {&cx, &cy, &cx, &cy, &radius};
    PointOnCircleFunction shared(sharedVars);
    expectGradientsNear(shared.gradient(), finiteDifferenceGradient(shared, sharedVars), sharedVars);
}

TEST(LineLineParallelFunctionTest, Evaluate) {
    double x1 = 0, y1 = 0, x2 = 1, y2 = 0;
    double x3 = 0, y3 = 1, x4 = 1, y4 = 1;
    std::vector<VAR> vars = {&x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4};
    LineLineParallelFunction f(vars);
    EQ(f.evaluate(), 0.0);
}

// ======== LineLinePerpendicularFunction ========
TEST(LineLinePerpendicularFunctionTest, Evaluate) {
    double x1 = 0, y1 = 0, x2 = 1, y2 = 0;
    double x3 = 0, y3 = 0, x4 = 0, y4 = 1;
    std::vector<VAR> vars = {&x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4};
    LineLinePerpendicularFunction f(vars);
    EQ(f.evaluate(), 0.0);
}

TEST(LineLineAngleFunctionTest, GradientMatchesFiniteDifference) {
    double x1 = 1, y1 = 2, x2 = 4, y2 = 3;
    double x3 = -2, y3 = 1, x4 = 0, y4 = 5;
    std::vector<VAR> vars = {&x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4};

    LineLineAngleFunction f(vars, 0.75);
    expectGradientsNear(f.gradient(), finiteDifferenceGradient(f, vars), vars, 2e-5);
}

TEST(SharedPointGradientTest, LineConstraintsAccumulateBothContributions) {
    double ax = 1, ay = 2, bx = 4, by = 3, cx = 6, cy = 7;
    const std::vector<VAR> vars = {&ax, &ay, &bx, &by, &bx, &by, &cx, &cy};

    LineLineParallelFunction parallel(vars);
    LineLinePerpendicularFunction perpendicular(vars);
    LineLineAngleFunction angle(vars, 0.75);
    expectGradientsNear(parallel.gradient(), finiteDifferenceGradient(parallel, vars), vars);
    expectGradientsNear(perpendicular.gradient(), finiteDifferenceGradient(perpendicular, vars), vars);
    expectGradientsNear(angle.gradient(), finiteDifferenceGradient(angle, vars), vars, 2e-5);
}

TEST(SharedPointGradientTest, PointOnLineAccumulatesEndpointContribution) {
    double ax = 1, ay = 2, bx = 4, by = 3;
    const std::vector<VAR> vars = {&ax, &ay, &ax, &ay, &bx, &by};
    PointOnLineFunction onLine(vars);
    expectGradientsNear(onLine.gradient(), finiteDifferenceGradient(onLine, vars), vars);
}

TEST(SharedPointGradientTest, ArcCenterAccumulatesEndpointContribution) {
    double ax = 1, ay = 2, bx = 4, by = 3;
    const std::vector<VAR> vars = {&ax, &ay, &bx, &by, &ax, &ay};
    ArcCenterOnPerpendicularFunction center(vars);
    expectGradientsNear(center.gradient(), finiteDifferenceGradient(center, vars), vars);
}

// ======== VerticalFunction ========
TEST(VerticalFunctionTest, EvaluateVerticalLine) {
    double x1 = 2, y1 = 0, x2 = 2, y2 = 5;
    std::vector<VAR> vars = {&x1, &y1, &x2, &y2};
    VerticalFunction f(vars);
    EQ(f.evaluate(), 0.0); // dx = 0
}

// ======== HorizontalFunction ========
TEST(HorizontalFunctionTest, EvaluateHorizontalLine) {
    double x1 = 0, y1 = 0, x2 = 5, y2 = 0;
    std::vector<VAR> vars = {&x1, &y1, &x2, &y2};
    HorizontalFunction f(vars);
    EQ(f.evaluate(), 0.0); // dy = 0
}

// ======== ArcCenterOnPerpendicularFunction ========
TEST(ArcCenterOnPerpendicularFunctionTest, Evaluate) {
    double Ax = 0, Ay = 0, Bx = 2, By = 0, Cx = 1, Cy = 1;
    std::vector<VAR> vars = {&Ax, &Ay, &Bx, &By, &Cx, &Cy};
    ArcCenterOnPerpendicularFunction f(vars);
    EQ(f.evaluate(), 0.0); // центр лежит на перпендикуляре
}

TEST(DegenerateRequirementFunctionTest, PointLineUsesDistanceToCollapsedEndpoint) {
    double px = 100, py = 100, ax = 0, ay = 0, bx = 0, by = 0;
    const std::vector<VAR> vars = {&px, &py, &ax, &ay, &bx, &by};
    PointOnLineFunction onLine(vars);
    PointLineDistanceFunction distance(vars, 5);
    EQ(onLine.evaluate(), std::hypot(100.0, 100.0));
    EQ(distance.evaluate(), std::hypot(100.0, 100.0) - 5);
    const auto gradient = onLine.gradient();
    EQ(gradient.at(&px), 1 / std::sqrt(2.0));
    EQ(gradient.at(&py), 1 / std::sqrt(2.0));
    EQ(gradient.at(&ax), -1 / std::sqrt(2.0));
    EQ(gradient.at(&bx), 0);

    px = 3; py = 4;
    EQ(distance.evaluate(), 0);
    px = 0; py = 0;
    EQ(onLine.evaluate(), 0);
    EQ(distance.evaluate(), -5);
    for (const auto& [variable, derivative] : onLine.gradient()) EQ(derivative, 0);

    // Shared endpoint pointers must accumulate the point and endpoint contributions.
    PointOnLineFunction shared({&ax, &ay, &ax, &ay, &ax, &ay});
    EQ(shared.evaluate(), 0);
    for (const auto& [variable, derivative] : shared.gradient()) EQ(derivative, 0);
}

TEST(DegenerateRequirementFunctionTest, DirectionAndArcChordAreUndefinedAtEitherCollapsedLine) {
    for (double length : {0.0, 5e-13, 1e-12}) {
        for (int collapsed : {0, 1, 2}) {
            SCOPED_TRACE(length);
            SCOPED_TRACE(collapsed);
            std::array<double, 8> coordinates{0, 0, 2, 0, 0, 0, 0, 2};
            if (collapsed != 1) coordinates[2] = length;
            if (collapsed != 0) coordinates[7] = length;
            std::vector<VAR> vars;
            for (auto& coordinate : coordinates) vars.push_back(&coordinate);
            LineLineParallelFunction parallel(vars);
            LineLinePerpendicularFunction perpendicular(vars);
            LineLineAngleFunction angle(vars, 1);
            for (const RequirementFunction* function :
                 std::array<const RequirementFunction*, 3>{&parallel, &perpendicular, &angle}) {
                EXPECT_FALSE(std::isfinite(function->evaluate()));
                for (const auto& [variable, derivative] : function->gradient()) EQ(derivative, 0);
            }
        }
        double ax = 0, ay = 0, bx = length, by = 0, cx = 0, cy = 2;
        const std::vector<VAR> vars = {&ax, &ay, &bx, &by};
        VerticalFunction vertical(vars);
        HorizontalFunction horizontal(vars);
        ArcCenterOnPerpendicularFunction arc({&ax, &ay, &bx, &by, &cx, &cy});
        for (const RequirementFunction* function :
             std::array<const RequirementFunction*, 3>{&vertical, &horizontal, &arc}) {
            EXPECT_FALSE(std::isfinite(function->evaluate()));
            for (const auto& [variable, derivative] : function->gradient()) EQ(derivative, 0);
        }
    }
}

TEST(DegenerateRequirementFunctionTest, ShortNonzeroLinesCannotHideOrientationErrors) {
    double ax = 0, ay = 0, bx = 5e-11, by = 0;
    double cx = 0, cy = 0, dx = 0, dy = 5e-11;
    const std::vector<VAR> vars = {&ax, &ay, &bx, &by, &cx, &cy, &dx, &dy};
    LineLineParallelFunction parallel(vars);
    LineLinePerpendicularFunction perpendicular(vars);
    EQ(parallel.evaluate(), 1);
    EQ(perpendicular.evaluate(), 0);
    dx = dy; dy = 0;
    EQ(parallel.evaluate(), 0);
    EQ(perpendicular.evaluate(), 1);
    LineLineAngleFunction angle(vars, 1);
    EQ(angle.evaluate(), 1 - std::cos(1.0));
}

TEST(ArcCenterOnPerpendicularFunctionTest, CenterAtMidpointOfNonzeroChordIsDefined) {
    double ax = -2, ay = 0, bx = 2, by = 0, cx = 0, cy = 0;
    ArcCenterOnPerpendicularFunction arc({&ax, &ay, &bx, &by, &cx, &cy});
    EQ(arc.evaluate(), 0);
    EXPECT_TRUE(std::isfinite(arc.gradient().at(&cx)));
}

TEST(ArcCenterOnPerpendicularFunctionTest, ShortChordCannotHideOffBisectorCenter) {
    double ax = 0, ay = 0, bx = 5e-11, by = 0, cx = 100, cy = 100;
    ArcCenterOnPerpendicularFunction arc({&ax, &ay, &bx, &by, &cx, &cy});
    EQ(arc.evaluate(), cx - 0.5 * bx);
    cx = 0.5 * bx;
    EQ(arc.evaluate(), 0);
}

TEST(DegenerateRequirementFunctionTest, CollapsedSegmentCanLieOnCircleAndCoincidentPointsCanSatisfyZeroDistance) {
    double ax = 3, ay = 4, bx = 3, by = 4, cx = 0, cy = 0, radius = 5;
    LineOnCircleFunction onCircle({&ax, &ay, &bx, &by, &cx, &cy, &radius});
    EQ(onCircle.evaluate(), 0);
    ax = 0; ay = 0; bx = 0; by = 0;
    EXPECT_GT(onCircle.evaluate(), 0);
    PointOnPointFunction coincidence({&ax, &ay, &bx, &by});
    PointPointDistanceFunction zeroDistance({&ax, &ay, &bx, &by}, 0);
    PointPointDistanceFunction positiveDistance({&ax, &ay, &bx, &by}, 1);
    EQ(coincidence.evaluate(), 0);
    EQ(zeroDistance.evaluate(), 0);
    EQ(positiveDistance.evaluate(), -1);
}
