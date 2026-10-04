#ifndef OURPAINTDCM_FUNCTION_MATHFUNCTION_H
#define OURPAINTDCM_FUNCTION_MATHFUNCTION_H
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "Enums.h"
#include "ID.h"
#include "NumericValidation.h"

#define VAR double*

namespace OurPaintDCM::Function {

    /**
     * @brief Abstract base class for all geometric constraint functions.
     *
     * Each requirement function represents a mathematical constraint f(x),
     * used in geometric solvers or optimization systems.
     *
     * The function typically computes:
     *  - evaluate(): deviation from the desired condition
     *  - gradient(): partial derivatives with respect to all variables
     */
    class RequirementFunction {
    protected:
        double _weight = 1.0;                ///< Importance coefficient in the optimization system
        std::vector<VAR> _vars;              ///< List of variable pointers
        Utils::RequirementType _t;           ///< Type of the geometric requirement

    public:
        RequirementFunction(Utils::RequirementType type, const std::vector<VAR>& vars)
            : _t(type), _vars(vars) {
            for (VAR variable : vars) {
                if (variable == nullptr) throw std::invalid_argument("Constraint variable must not be null");
                Utils::requireFinite(*variable);
            }
            if ((type == Utils::RequirementType::ET_LINECIRCLEDIST && vars.size() == 7) ||
                (type == Utils::RequirementType::ET_LINEONCIRCLE && (vars.size() == 5 || vars.size() == 7))) {
                Utils::requirePositiveRadius(*vars.back());
            }
        }

        virtual ~RequirementFunction() = default;

        /// Compute the scalar value of the constraint (error).
        virtual double evaluate() const = 0;

        /// Compute the gradient (first-order derivatives) with respect to variables.
        virtual std::unordered_map<VAR, double> gradient() const = 0;

        /// Return variables involved in this constraint.
        virtual std::vector<VAR> getVars() const {
            return _vars;
        }

        /// Return the number of variables used.
        virtual size_t getVarCount() const = 0;

        /**
         * @brief Try to expose this requirement as a direct assignment var = value.
         *
         * Default implementation returns false. Requirements like FixCoordinateFunction
         * override it to provide algebraic elimination metadata.
         */
        virtual bool tryGetAssignment(VAR& var, double& value) const {
            (void)var;
            (void)value;
            return false;
        }

        /// Return the type of this geometric requirement.
        virtual Utils::RequirementType getType() const {
            return _t;
        }

        /// Return the current weight of this function.
        double getWeight() const {
            return _weight;
        }

        /// Set the weight for this function.
        void setWeight(double w) {
            if (!std::isfinite(w) || w < 0.0) {
                throw std::invalid_argument("Requirement weight must be finite and non-negative");
            }
            _weight = w;
        }
    };

    /**
     * @brief Fixed distance between a point and a line segment.
     *
     * Variables: [Px, Py, L1x, L1y, L2x, L2y]
     */
    class PointLineDistanceFunction : public RequirementFunction {
        double _distance;
    public:
        PointLineDistanceFunction(const std::vector<VAR>& vars, double dist);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Point must lie exactly on a line segment (distance == 0).
     *
     * Variables: [Px, Py, L1x, L1y, L2x, L2y]
     */
    class PointOnLineFunction : public RequirementFunction {
    public:
        PointOnLineFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Fixed distance between two points.
     *
     * Variables: [P1x, P1y, P2x, P2y]
     */
    class PointPointDistanceFunction : public RequirementFunction {
        double _distance;
    public:
        PointPointDistanceFunction(const std::vector<VAR>& vars, double dist);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Two points must coincide (distance == 0).
     *
     * Variables: [P1x, P1y, P2x, P2y]
     */
    class PointOnPointFunction : public RequirementFunction {
    public:
        PointOnPointFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Fixed external clearance: distance(center, segment) - radius - target.
     *
     * Variables: [L1x, L1y, L2x, L2y, Cx, Cy, R]
     */
    class LineCircleDistanceFunction : public RequirementFunction {
        double _distance;
    public:
        LineCircleDistanceFunction(const std::vector<VAR>& vars, double dist);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /// One endpoint of a line must lie on its circle. Variables: [Px, Py, Cx, Cy, R].
    class PointOnCircleFunction : public RequirementFunction {
    public:
        explicit PointOnCircleFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Norm of the two endpoint residuals; zero only when both lie on the circle.
     * RequirementSystem uses two PointOnCircleFunction rows for solving and diagnostics.
     *
     * Variables: [L1x, L1y, L2x, L2y, Cx, Cy, R]
     */
    class LineOnCircleFunction : public RequirementFunction {
    public:
        LineOnCircleFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Two line segments must be parallel.
     *
     * Variables: [A1x, A1y, A2x, A2y, B1x, B1y, B2x, B2y]
     */
    class LineLineParallelFunction : public RequirementFunction {
    public:
        LineLineParallelFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Two line segments must be perpendicular.
     *
     * Variables: [A1x, A1y, A2x, A2y, B1x, B1y, B2x, B2y]
     */
    class LineLinePerpendicularFunction : public RequirementFunction {
    public:
        LineLinePerpendicularFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Fixed angle between two line segments.
     *
     * Variables: [A1x, A1y, A2x, A2y, B1x, B1y, B2x, B2y]
     */
    class LineLineAngleFunction : public RequirementFunction {
        double _angle;
    public:
        LineLineAngleFunction(const std::vector<VAR>& vars, double angle);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Line must be vertical (aligned with the Y-axis).
     *
     * Variables: [L1x, L1y, L2x, L2y]
     */
    class VerticalFunction : public RequirementFunction {
    public:
        VerticalFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Line must be horizontal (aligned with the X-axis).
     *
     * Variables: [L1x, L1y, L2x, L2y]
     */
    class HorizontalFunction : public RequirementFunction {
    public:
        HorizontalFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Default constraint used for arcs: the arc center lies on a perpendicular bisector.
     *
     * Variables: [P1x, P1y, P2x, P2y, Cx, Cy]
     */
    class ArcCenterOnPerpendicularFunction : public RequirementFunction {
    public:
        ArcCenterOnPerpendicularFunction(const std::vector<VAR>& vars);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
    };

    /**
     * @brief Fixes a single coordinate at a target value.
     *
     * Used as a building block for fix-point, fix-line, and fix-circle constraints.
     * Each instance pins one scalar variable: evaluate() = *var - target.
     *
     * Variables: [coordinate]
     */
    class FixCoordinateFunction : public RequirementFunction {
        double _target;
    public:
        FixCoordinateFunction(Utils::RequirementType type, const std::vector<VAR>& vars, double target);
        double evaluate() const override;
        std::unordered_map<VAR, double> gradient() const override;
        size_t getVarCount() const override;
        bool tryGetAssignment(VAR& var, double& value) const override;
    };
}

#endif // OURPAINTDCM_FUNCTION_MATHFUNCTION_H
