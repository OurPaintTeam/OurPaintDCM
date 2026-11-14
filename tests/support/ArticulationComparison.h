#ifndef OURPAINTDCM_TESTS_SUPPORT_ARTICULATIONCOMPARISON_H
#define OURPAINTDCM_TESTS_SUPPORT_ARTICULATIONCOMPARISON_H

#include "DCMManager.h"
#include "SolvePipeline.h"
#include "sparse/SparseLevenbergMarquardtSolver.h"
#include <cmath>
#include <string>

namespace ArticulationComparison {
using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

inline bool satisfied(const DCMManager& manager) {
    for (const auto& binding : manager.getRequirementSystem().getFunctions())
        if (!binding->mathematical()->satisfied(1e-6)) return false;
    return true;
}

inline std::vector<ID> intersection(DCMManager& manager, bool feasible) {
    const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto p = manager.addFigure(FigureDescriptor::point(3, 1));
    const auto b = manager.addFigure(FigureDescriptor::point(10, 0));
    manager.addRequirement(RequirementDescriptor::fixPoint(a));
    manager.addRequirement(RequirementDescriptor::fixPoint(b));
    manager.addRequirement(RequirementDescriptor::pointPointDist(a, p, feasible ? 5 : 2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p, b, feasible ? 7 : 3));
    manager.setSolveMode(SolveMode::LOCAL);
    return {a, p, b};
}

struct Model {
    DCMManager manager;
    std::vector<ID> points;
    Model(std::size_t count, const std::string& shape, bool anchored = true) {
        for (std::size_t i = 0; i < count; ++i)
            points.push_back(manager.addFigure(FigureDescriptor::point(5 * i + (i ? 0.2 : 0), 0.1 * (i % 2))));
        if (anchored) manager.addRequirement(RequirementDescriptor::fixPoint(points[0]));
        for (std::size_t i = 1; i < count; ++i) {
            const auto parent = shape == "star" ? 0 : i - 1;
            manager.addRequirement(RequirementDescriptor::pointPointHorizontalDist(points[parent], points[i], 5 * (i - parent)));
            manager.addRequirement(RequirementDescriptor::pointPointHorizontal(points[parent], points[i]));
        }
        if (shape == "cycle")
            manager.addRequirement(RequirementDescriptor::pointPointDist(points.front(), points.back(), 5 * (count - 1)));
        manager.setSolveMode(SolveMode::LOCAL);
    }
    void perturb(double amount) {
        const auto last = manager.getFigure(points.back()).value();
        manager.updatePoint({points.back(), *last.x + amount, *last.y + 0.1});
    }
    bool solve() { return manager.solve(manager.getComponentForFigure(points.front()), 1e-6); }
    bool satisfied() const { return ArticulationComparison::satisfied(manager); }
};

// Ordinary full-component preparation and optimizer, with the same tolerances as DCMManager.
struct JointSolve {
    DCMManager& manager;
    System::SolvePipeline pipeline;
    SparseLMSolver optimizer{100, 1e-3, 1e-8, 1e-8, 1e-14};
    explicit JointSolve(Model& model) : JointSolve(model.manager) {}
    explicit JointSolve(DCMManager& manager, const std::unordered_set<VAR>& locked = {}) : manager(manager) {
        const auto& functions = manager.getRequirementSystem().getFunctions();
        const auto fixed = System::collectFixedAssignments(functions);
        for (const auto& [variable, target] : fixed) *variable = target;
        pipeline = System::buildSolvePipeline(functions, fixed, locked, SolverType::LEVENBERG_MARQUARDT);
    }
    bool solve() {
        if (pipeline.task) {
            optimizer.setTask(pipeline.task.get());
            optimizer.optimize();
        }
        return ArticulationComparison::satisfied(manager);
    }
};
} // namespace ArticulationComparison

#endif
