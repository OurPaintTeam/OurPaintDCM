#include "DCMManager.h"
#include "ErrorFunction.h"
#include "SparseQR.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

int main(int argc, char** argv) {
    std::cout << std::boolalpha;
    if (argc > 1 && (std::string(argv[1]) == "sparse-copy" || std::string(argv[1]) == "sparse-move")) {
        Matrix<> dense{{2.0, 0.0}, {0.0, 3.0}};
        SparseQR original{SparseMatrix<>(dense)};
        original.qr();
        Matrix<> rhs(2, 1);
        rhs(0, 0) = 2.0; rhs(1, 0) = 3.0;
        const auto expected = original.solve(rhs, 0.0);
        std::cout << "sparse.original.solution=" << expected(0, 0) << ',' << expected(1, 0) << std::endl;
        SparseQR copy = std::string(argv[1]) == "sparse-copy" ? SparseQR(original) : SparseQR(std::move(original));
        const auto actual = copy.solve(rhs, 0.0);
        std::cout << "sparse.transferred.solution=" << actual(0, 0) << ',' << actual(1, 0) << std::endl;
        return 0;
    }
    {
        DCMManager d;
        const auto line = d.addFigure(FigureDescriptor::line(0, 0, 10, 0));
        d.addRequirement(RequirementDescriptor::horizontal(line));
        const auto point = d.getFigure(line)->pointIds[0];
        const auto state = d.snapshot();
        std::cout << "components.before_restore=" << d.getComponentCount() << '\n';
        d.restoreSnapshot(state);
        std::cout << "components.after_restore=" << d.getComponentCount() << '\n';
        d.setSolveMode(SolveMode::DRAG);
        d.updatePoint({point, 0, 5});
        const auto coords = d.getFigure(line)->coords;
        std::cout << "drag.after_restore.horizontal_delta=" << coords[3] - coords[1] << '\n';
    }
    {
        DCMManager d;
        const auto a = d.addFigure(FigureDescriptor::point(0, 0));
        const auto b = d.addFigure(FigureDescriptor::point(1, 0));
        d.addRequirement(RequirementDescriptor::fixPoint(a));
        d.addRequirement(RequirementDescriptor::fixPoint(b));
        d.addRequirement(RequirementDescriptor::pointPointDist(a, b, 2));
        std::cout << "infeasible.fixed.solve=" << d.solve() << '\n';
        std::cout << "infeasible.fixed.residual_norm=" << d.getRequirementSystem().residuals().norm() << '\n';
    }
    {
        DCMManager d;
        const auto a = d.addFigure(FigureDescriptor::point(0, 0));
        const auto b = d.addFigure(FigureDescriptor::point(1.5, 0));
        d.addRequirement(RequirementDescriptor::pointPointDist(a, b, 1));
        d.addRequirement(RequirementDescriptor::pointPointDist(a, b, 2));
        std::cout << "infeasible.free.solve=" << d.solve() << '\n';
        std::cout << "infeasible.free.residual_norm=" << d.getRequirementSystem().residuals().norm() << '\n';
    }
    {
        DCMManager d;
        const auto a = d.addFigure(FigureDescriptor::point(0, 0));
        const auto b = d.addFigure(FigureDescriptor::point(0, 0));
        d.addRequirement(RequirementDescriptor::pointPointDist(a, b, 5));
        std::cout << "stationary.unsatisfied.solve=" << d.solve() << '\n';
        std::cout << "stationary.unsatisfied.residual_norm=" << d.getRequirementSystem().residuals().norm() << '\n';
    }
    {
        DCMManager d;
        const auto a = d.addFigure(FigureDescriptor::point(0, 0));
        const auto b = d.addFigure(FigureDescriptor::point(1, 0));
        d.addRequirement(RequirementDescriptor::pointPointDist(a, b, 1));
        const Eigen::MatrixXd before(d.getRequirementSystem().J());
        d.updatePoint({b, 0, 1});
        const Eigen::MatrixXd cached(d.getRequirementSystem().J());
        d.requirementSystem().updateJ();
        const Eigen::MatrixXd refreshed(d.getRequirementSystem().J());
        std::cout << "jacobian.cached_change=" << (cached - before).norm() << '\n';
        std::cout << "jacobian.stale_error=" << (cached - refreshed).norm() << '\n';
    }
    {
        double v[] = {0, 0, 1, 0, 0, 0, 0, 1};
        std::vector<double*> refs;
        std::vector<Variable*> variables;
        for (double& x : v) { refs.push_back(&x); variables.push_back(new Variable(&x)); }
        OurPaintDCM::Function::LineLineAngleFunction diagnostic(refs, 90);
        SectionSectionAngleError solving(variables, 90);
        std::cout << "angle.90deg.diagnostic=" << diagnostic.evaluate() << '\n';
        std::cout << "angle.90deg.solving=" << solving.evaluate() << '\n';
    }
    {
        double v[] = {0, 0, 2, 0, 1, 3, 1};
        std::vector<double*> refs;
        std::vector<Variable*> variables;
        for (double& x : v) { refs.push_back(&x); variables.push_back(new Variable(&x)); }
        OurPaintDCM::Function::LineCircleDistanceFunction diagnostic(refs, 2);
        SectionCircleDistanceError solving(variables, 2);
        std::cout << "circle.gap2.target2.diagnostic=" << diagnostic.evaluate() << '\n';
        const double oldValue = solving.evaluate();
        v[5] = 30; v[6] = 7;
        std::cout << "circle.solving.change_after_center_radius=" << solving.evaluate() - oldValue << '\n';
    }
    {
        double v[] = {5, 0, 15, 0, 0, 0, 10};
        std::vector<double*> refs;
        for (double& x : v) refs.push_back(&x);
        OurPaintDCM::Function::LineOnCircleFunction f(refs);
        std::cout << "circle.endpoints5_and15.radius10.residual=" << f.evaluate() << '\n';
    }
    {
        double v[] = {0, 0, 2, 0, 3, 1, 1};
        std::vector<double*> refs;
        for (double& x : v) refs.push_back(&x);
        OurPaintDCM::Function::LineCircleDistanceFunction f(refs, 0);
        const auto g = f.gradient();
        const double h = 1e-6;
        v[0] = h; const double plus = f.evaluate();
        v[0] = -h; const double minus = f.evaluate();
        std::cout << "circle.endpoint_projection.analytic_dx1=" << g.at(&v[0]) << '\n';
        std::cout << "circle.endpoint_projection.numeric_dx1=" << (plus - minus) / (2 * h) << '\n';
    }
    {
        DCMManager d;
        const auto c = d.addFigure(FigureDescriptor::circle(0, 0, 1));
        d.updateCircle({c, -3});
        std::cout << "validation.negative_radius=" << *d.getFigure(c)->radius << '\n';
        const auto p = d.addFigure(FigureDescriptor::point(std::numeric_limits<double>::quiet_NaN(), 0));
        std::cout << "validation.nan_point_accepted=" << d.hasFigure(p) << '\n';
    }
    {
        DCMManager d;
        d.addFigure(FigureDescriptor::arc(1, 0, 0, 2, 0, 0));
        std::cout << "arc.unequal_radii.requirements=" << d.requirementCount() << '\n';
        std::cout << "arc.unequal_radii.solve=" << d.solve() << '\n';
    }
    {
        DCMManager d;
        const auto p = d.addFigure(FigureDescriptor::point(0, 0));
        const auto q = d.addFigure(FigureDescriptor::point(1, 0));
        const auto r = d.addFigure(FigureDescriptor::point(0, 1));
        const auto a = d.addFigure(FigureDescriptor::line(p, q));
        const auto b = d.addFigure(FigureDescriptor::line(p, r));
        d.removeFigure(a, true);
        std::cout << "cascade.shared_neighbor_survives=" << d.hasFigure(b) << '\n';
    }
    {
        DCMManager d;
        const auto line = d.addFigure(FigureDescriptor::line(0, 0, 1, 0));
        const auto& supposedlyReadOnly = d.getStorage();
        supposedlyReadOnly.get<Figures::Line2D>(line)->p1->x() = 123;
        std::cout << "const_storage.point_mutated=" << d.getFigure(line)->coords[0] << '\n';
    }
    {
        double ax = 0, ay = 0, bx = 1, by = 0, cx = 0, cy = 1;
        OurPaintDCM::Function::LineLineParallelFunction f({&ax, &ay, &bx, &by, &ax, &ay, &cx, &cy});
        const auto g = f.gradient();
        const double h = 1e-6;
        ax = h; const double plus = f.evaluate();
        ax = -h; const double minus = f.evaluate();
        std::cout << "alias.parallel_shared_point.analytic_dx=" << g.at(&ax) << '\n';
        std::cout << "alias.parallel_shared_point.numeric_dx=" << (plus - minus) / (2 * h) << '\n';
    }
    {
        DCMManager d;
        const auto a = d.addFigure(FigureDescriptor::point(0, 0));
        const auto b = d.addFigure(FigureDescriptor::point(1, 0));
        d.addRequirement(RequirementDescriptor::pointPointDist(a, b, 1));
        d.addRequirement(RequirementDescriptor::pointPointDist(a, b, 2));
        std::cout << "graph.two_constraints.edge_count=" << d.getRequirementSystem().buildDependencyGraph().edgeCount() << '\n';
    }
    {
        DCMManager d;
        const auto fixed = d.addFigure(FigureDescriptor::point(0, 0));
        d.addFigure(FigureDescriptor::point(10, 10));
        d.addRequirement(RequirementDescriptor::fixPoint(fixed));
        std::cout << "diagnosis.fixed_plus_free_point.well_constrained=" <<
            (d.getRequirementSystem().diagnose() == SystemStatus::WELL_CONSTRAINED) << '\n';
    }
    {
        double ax = 0, ay = 0, bx = 1, by = 0;
        System::RequirementFunctionSystem system;
        auto f = std::make_shared<OurPaintDCM::Function::PointPointDistanceFunction>(
            std::vector<double*>{&ax, &ay, &bx, &by}, 1);
        f->setWeight(2);
        system.addFunction(f);
        const double h = 1e-6;
        ax = h; const double plus = system.residuals()[0];
        ax = -h; const double minus = system.residuals()[0];
        ax = 0;
        std::cout << "weights.analytic_dx=" << system.J().coeff(0, 0) << '\n';
        std::cout << "weights.numeric_dx=" << (plus - minus) / (2 * h) << '\n';
    }
    std::cout << "manager.is_move_constructible=" << std::is_move_constructible_v<DCMManager> << '\n';
    std::cout << "manager.is_move_assignable=" << std::is_move_assignable_v<DCMManager> << '\n';
}
