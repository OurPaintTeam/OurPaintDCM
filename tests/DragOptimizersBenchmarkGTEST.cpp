#include <gtest/gtest.h>

#include "DCMManager.h"

#include "GradientOptimizer.h"
#include "AdamOptimizer.h"
#include "LMWithSparse.h"
#include "LSMTask.h"
#include "LSMFORLMTask.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {

class RequirementDerivativeAdapter;

class RequirementFunctionAdapter : public Function {
    std::shared_ptr<OurPaintDCM::Function::RequirementFunction> req_;
public:
    explicit RequirementFunctionAdapter(std::shared_ptr<OurPaintDCM::Function::RequirementFunction> req)
        : req_(std::move(req)) {}

    double evaluate() const override { return req_->evaluate(); }
    ::Function* derivative(Variable* var) const override;
    ::Function* clone() const override { return new RequirementFunctionAdapter(req_); }
    std::string to_string() const override { return "ReqFunc"; }
};

class RequirementDerivativeAdapter : public Function {
    std::shared_ptr<OurPaintDCM::Function::RequirementFunction> req_;
    double* var_;
public:
    RequirementDerivativeAdapter(std::shared_ptr<OurPaintDCM::Function::RequirementFunction> req, double* var)
        : req_(std::move(req)), var_(var) {}

    double evaluate() const override {
        auto grad = req_->gradient();
        auto it = grad.find(var_);
        return (it != grad.end()) ? it->second : 0.0;
    }
    ::Function* derivative(Variable*) const override { return new Constant(0.0); }
    ::Function* clone() const override { return new RequirementDerivativeAdapter(req_, var_); }
    std::string to_string() const override { return "ReqDeriv"; }
};

::Function* RequirementFunctionAdapter::derivative(Variable* var) const {
    return new RequirementDerivativeAdapter(req_, var->value);
}

struct BenchConfig {
    int objects;
    int requirements;
};

struct BenchResult {
    std::string name;
    long long buildMs;
    long long optimizeMs;
    bool converged;
    double error;
};

struct DragStepResult {
    std::string name;
    int steps;
    long long totalMs;
    double avgStepMs;
    int convergedSteps;
    double finalError;
};

std::vector<double> snapshotValues(const std::vector<VAR>& allVars) {
    std::vector<double> values;
    values.reserve(allVars.size());
    for (auto* v : allVars) values.push_back(*v);
    return values;
}

void restoreValues(const std::vector<VAR>& allVars, const std::vector<double>& values) {
    for (size_t i = 0; i < allVars.size(); ++i) {
        *allVars[i] = values[i];
    }
}

std::vector<Variable*> makeMathVars(const std::vector<VAR>& allVars) {
    std::vector<Variable*> vars;
    vars.reserve(allVars.size());
    for (auto* v : allVars) vars.push_back(new Variable(v));
    return vars;
}

std::vector<::Function*> makeMathFuncs(
    const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& reqFuncs) {
    std::vector<::Function*> funcs;
    funcs.reserve(reqFuncs.size());
    for (const auto& rf : reqFuncs) funcs.push_back(new RequirementFunctionAdapter(rf));
    return funcs;
}

BenchResult runGradient(const std::vector<VAR>& allVars,
                        const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& reqFuncs,
                        const std::vector<double>& savedValues) {
    restoreValues(allVars, savedValues);
    auto vars = makeMathVars(allVars);
    auto funcs = makeMathFuncs(reqFuncs);

    const auto t0 = std::chrono::high_resolution_clock::now();
    LSMTask task(funcs, vars);
    const auto t1 = std::chrono::high_resolution_clock::now();

    GradientOptimizer optimizer(0.01, 100);
    optimizer.setTask(&task);
    optimizer.optimize();
    const auto t2 = std::chrono::high_resolution_clock::now();

    for (auto* v : vars) delete v;

    return {
        "Gradient",
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(),
        std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count(),
        optimizer.isConverged(),
        optimizer.getCurrentError()
    };
}

BenchResult runAdam(const std::vector<VAR>& allVars,
                    const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& reqFuncs,
                    const std::vector<double>& savedValues) {
    restoreValues(allVars, savedValues);
    auto vars = makeMathVars(allVars);
    auto funcs = makeMathFuncs(reqFuncs);

    const auto t0 = std::chrono::high_resolution_clock::now();
    LSMFORLMTask task(funcs, vars);
    const auto t1 = std::chrono::high_resolution_clock::now();

    AdamOptimizer optimizer(0.001, 0.9, 0.999, 1e-8, 1e-6, 100);
    optimizer.setTask(&task);
    optimizer.optimize();
    const auto t2 = std::chrono::high_resolution_clock::now();

    for (auto* f : funcs) delete f;
    for (auto* v : vars) delete v;

    return {
        "Adam",
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(),
        std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count(),
        optimizer.isConverged(),
        optimizer.getCurrentError()
    };
}

BenchResult runLMSparse(const std::vector<VAR>& allVars,
                        const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& reqFuncs,
                        const std::vector<double>& savedValues) {
    restoreValues(allVars, savedValues);
    auto vars = makeMathVars(allVars);
    auto funcs = makeMathFuncs(reqFuncs);

    const auto t0 = std::chrono::high_resolution_clock::now();
    LSMFORLMTask task(funcs, vars);
    const auto t1 = std::chrono::high_resolution_clock::now();

    LMSparse optimizer(100);
    optimizer.setTask(&task);
    optimizer.optimize();
    const auto t2 = std::chrono::high_resolution_clock::now();

    for (auto* f : funcs) delete f;
    for (auto* v : vars) delete v;

    return {
        "LMSparse",
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(),
        std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count(),
        optimizer.isConverged(),
        optimizer.getCurrentError()
    };
}

void printResults(const BenchConfig& cfg, const std::vector<BenchResult>& results) {
    std::cout << "\n===== Drag Optimizer Benchmark =====\n";
    std::cout << "Objects: " << cfg.objects << ", Requirements: " << cfg.requirements << '\n';
    std::cout << std::left
              << std::setw(12) << "Optimizer"
              << std::setw(12) << "Build(ms)"
              << std::setw(12) << "Opt(ms)"
              << std::setw(12) << "Total(ms)"
              << std::setw(10) << "Conv"
              << "Error\n";
    for (const auto& r : results) {
        std::cout << std::left
                  << std::setw(12) << r.name
                  << std::setw(12) << r.buildMs
                  << std::setw(12) << r.optimizeMs
                  << std::setw(12) << (r.buildMs + r.optimizeMs)
                  << std::setw(10) << (r.converged ? "yes" : "no")
                  << r.error << '\n';
    }
    std::cout << "===================================\n";
}

void printDragStepResults(int objects, int requirements, double delta, int steps,
                          const std::vector<DragStepResult>& results) {
    std::cout << "\n===== Small-Shift Drag Scenario =====\n";
    std::cout << "Objects: " << objects
              << ", Requirements: " << requirements
              << ", Delta: " << delta
              << ", Steps: " << steps << '\n';
    std::cout << std::left
              << std::setw(12) << "Optimizer"
              << std::setw(10) << "Steps"
              << std::setw(12) << "Total(ms)"
              << std::setw(14) << "AvgStep(ms)"
              << std::setw(12) << "ConvSteps"
              << "FinalError\n";
    for (const auto& r : results) {
        std::cout << std::left
                  << std::setw(12) << r.name
                  << std::setw(10) << r.steps
                  << std::setw(12) << r.totalMs
                  << std::setw(14) << std::fixed << std::setprecision(3) << r.avgStepMs
                  << std::setw(12) << r.convergedSteps
                  << r.finalError << '\n';
    }
    std::cout << "=====================================\n";
}

template <typename OptimizerRunner>
DragStepResult runSmallShiftScenario(const std::string& name,
                                     const std::vector<VAR>& allVars,
                                     const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& reqFuncs,
                                     const std::vector<double>& savedValues,
                                     int steps,
                                     double delta,
                                     OptimizerRunner&& runOptimizerOnce) {
    restoreValues(allVars, savedValues);
    const auto t0 = std::chrono::high_resolution_clock::now();

    int convergedSteps = 0;
    double finalError = 0.0;
    for (int i = 0; i < steps; ++i) {
        *allVars.front() += delta; // emulate dragging one coordinate by +0.01
        const auto [converged, error] = runOptimizerOnce(allVars, reqFuncs);
        if (converged) ++convergedSteps;
        finalError = error;
    }

    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    const double avgMs = static_cast<double>(totalMs) / static_cast<double>(steps);
    return {name, steps, totalMs, avgMs, convergedSteps, finalError};
}

} // namespace

class DragOptimizersBenchmark : public ::testing::TestWithParam<BenchConfig> {
protected:
    DCMManager manager_;

    void buildSingleComponentModel(int objectCount, int requirementCount) {
        const int pointCount = objectCount / 2;
        const int lineCount = objectCount - pointCount;
        ASSERT_GE(pointCount, 3);
        ASSERT_GE(lineCount, 2);

        std::vector<ID> points;
        points.reserve(pointCount);

        for (int i = 0; i < pointCount; ++i) {
            const double x = static_cast<double>(i) * 10.0;
            const double y = (i % 3 == 0) ? 0.15 : ((i % 3 == 1) ? -0.12 : 0.08);
            points.push_back(manager_.addFigure(FigureDescriptor::point(x, y)));
        }

        std::vector<ID> lines;
        lines.reserve(lineCount);
        for (int i = 0; i < lineCount; ++i) {
            const ID p1 = points[i % pointCount];
            const ID p2 = points[(i + 1) % pointCount];
            lines.push_back(manager_.addFigure(FigureDescriptor::line(p1, p2)));
        }

        int created = 0;
        for (int i = 0; i < pointCount && created < requirementCount; ++i) {
            const ID p1 = points[i];
            const ID p2 = points[(i + 1) % pointCount];
            manager_.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
            ++created;
        }
        for (int i = 0; i < lineCount && created < requirementCount; ++i) {
            manager_.addRequirement(RequirementDescriptor::horizontal(lines[i]));
            ++created;
        }
        for (int i = 0; i + 1 < lineCount && created < requirementCount; ++i) {
            manager_.addRequirement(RequirementDescriptor::lineLineParallel(lines[i], lines[i + 1]));
            ++created;
        }

        ASSERT_EQ(created, requirementCount);
        ASSERT_EQ(manager_.figureCount(), static_cast<size_t>(objectCount));
        ASSERT_EQ(manager_.requirementCount(), static_cast<size_t>(requirementCount));
        ASSERT_EQ(manager_.getComponentCount(), 1U);
    }
};

TEST_P(DragOptimizersBenchmark, CompareAdamGradientAndLMSparse) {
    const BenchConfig cfg = GetParam();
    buildSingleComponentModel(cfg.objects, cfg.requirements);

    auto& system = manager_.requirementSystem();
    const auto& reqFuncs = system.getFunctions();
    const auto allVars = system.getAllVars();
    ASSERT_FALSE(reqFuncs.empty());
    ASSERT_FALSE(allVars.empty());

    const auto savedValues = snapshotValues(allVars);

    std::vector<BenchResult> results;
    results.reserve(3);
    results.push_back(runAdam(allVars, reqFuncs, savedValues));
    results.push_back(runGradient(allVars, reqFuncs, savedValues));
    results.push_back(runLMSparse(allVars, reqFuncs, savedValues));

    printResults(cfg, results);

    EXPECT_EQ(results.size(), 3U);
}

INSTANTIATE_TEST_SUITE_P(
    LargeButSafeModels,
    DragOptimizersBenchmark,
    ::testing::Values(
        BenchConfig{100, 50},
        BenchConfig{125, 75},
        BenchConfig{150, 100}
    )
);

TEST(DragOptimizersSmallShiftBenchmark, CompareOnRepeatedSmallShifts) {
    DCMManager manager;

    constexpr int objects = 100;
    constexpr int requirements = 75;
    constexpr int steps = 15;
    constexpr double delta = 0.01;

    const int pointCount = objects / 2;
    const int lineCount = objects - pointCount;

    std::vector<ID> points;
    points.reserve(pointCount);
    for (int i = 0; i < pointCount; ++i) {
        points.push_back(manager.addFigure(FigureDescriptor::point(static_cast<double>(i) * 10.0, 0.0)));
    }

    std::vector<ID> lines;
    lines.reserve(lineCount);
    for (int i = 0; i < lineCount; ++i) {
        lines.push_back(manager.addFigure(FigureDescriptor::line(points[i % pointCount], points[(i + 1) % pointCount])));
    }

    int created = 0;
    for (int i = 0; i < pointCount && created < requirements; ++i, ++created) {
        manager.addRequirement(RequirementDescriptor::pointPointDist(points[i], points[(i + 1) % pointCount], 10.0));
    }
    for (int i = 0; i < lineCount && created < requirements; ++i, ++created) {
        manager.addRequirement(RequirementDescriptor::horizontal(lines[i]));
    }
    for (int i = 0; i + 1 < lineCount && created < requirements; ++i, ++created) {
        manager.addRequirement(RequirementDescriptor::lineLineParallel(lines[i], lines[i + 1]));
    }

    ASSERT_EQ(created, requirements);
    ASSERT_EQ(manager.getComponentCount(), 1U);

    auto& system = manager.requirementSystem();
    const auto& reqFuncs = system.getFunctions();
    const auto allVars = system.getAllVars();
    ASSERT_FALSE(reqFuncs.empty());
    ASSERT_FALSE(allVars.empty());
    const auto savedValues = snapshotValues(allVars);

    auto runGradientOnce = [](const std::vector<VAR>& vars,
                              const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& funcs) {
        auto mVars = makeMathVars(vars);
        auto mFuncs = makeMathFuncs(funcs);
        LSMTask task(mFuncs, mVars);
        GradientOptimizer optimizer(0.01, 100);
        optimizer.setTask(&task);
        optimizer.optimize();
        const bool converged = optimizer.isConverged();
        const double error = optimizer.getCurrentError();
        for (auto* v : mVars) delete v;
        return std::make_pair(converged, error);
    };

    auto runAdamOnce = [](const std::vector<VAR>& vars,
                          const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& funcs) {
        auto mVars = makeMathVars(vars);
        auto mFuncs = makeMathFuncs(funcs);
        LSMFORLMTask task(mFuncs, mVars);
        AdamOptimizer optimizer(0.001, 0.9, 0.999, 1e-8, 1e-6, 100);
        optimizer.setTask(&task);
        optimizer.optimize();
        const bool converged = optimizer.isConverged();
        const double error = optimizer.getCurrentError();
        for (auto* f : mFuncs) delete f;
        for (auto* v : mVars) delete v;
        return std::make_pair(converged, error);
    };

    auto runLMSparseOnce = [](const std::vector<VAR>& vars,
                              const std::vector<std::shared_ptr<OurPaintDCM::Function::RequirementFunction>>& funcs) {
        auto mVars = makeMathVars(vars);
        auto mFuncs = makeMathFuncs(funcs);
        LSMFORLMTask task(mFuncs, mVars);
        LMSparse optimizer(100);
        optimizer.setTask(&task);
        optimizer.optimize();
        const bool converged = optimizer.isConverged();
        const double error = optimizer.getCurrentError();
        for (auto* f : mFuncs) delete f;
        for (auto* v : mVars) delete v;
        return std::make_pair(converged, error);
    };

    std::vector<DragStepResult> results;
    results.reserve(3);
    results.push_back(runSmallShiftScenario("Adam", allVars, reqFuncs, savedValues, steps, delta, runAdamOnce));
    results.push_back(runSmallShiftScenario("Gradient", allVars, reqFuncs, savedValues, steps, delta, runGradientOnce));
    results.push_back(runSmallShiftScenario("LMSparse", allVars, reqFuncs, savedValues, steps, delta, runLMSparseOnce));

    printDragStepResults(objects, requirements, delta, steps, results);

    EXPECT_EQ(results.size(), 3U);
}
