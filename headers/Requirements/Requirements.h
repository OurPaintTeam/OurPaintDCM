#ifndef OURPAINTDCM_HEADERS_REQUIREMENTS_REQUIREMENTS_H
#define OURPAINTDCM_HEADERS_REQUIREMENTS_REQUIREMENTS_H
#include "Enums.h"
#include "Point2D.h"
#include "Line.h"
#include "ErrorFunction.h"

namespace OurPaintDCM::Requirements {
/**
* @brief Base class for all geometric requirements (constraints).
*
* A requirement represents a geometric constraint between objects
* (points, lines, circles, etc.). Each requirement can be converted
* into a mathematical function (equation) that must be satisfied
* by the solver.
*/
class Requirement {
protected:
    RequirementType type; ///< Type of the requirement (distance, angle, coincidence, etc.)
public:
    /**
    * @brief Construct a new Requirement.
    * @param t The type of this requirement.
    */
    explicit Requirement(RequirementType t)
        : type(t) {
    }

    /// Virtual destructor.
    virtual ~Requirement() = default;
    /**
     * @brief Get the type of this requirement.
     * @return The requirement type.
     */
    RequirementType getType() const noexcept { return type; }
    /**
     * @brief Convert this requirement into a mathematical function.
     *
     * Each requirement generates one function
     * (equations) that the global solver must satisfy.
     *
     * @return Pointer to a Function object representing this requirement.
     */
    virtual ErrorFunction* toFunction() = 0;
};

/**
 * @brief Requirement: fixed distance between point and sections.
 *
 * This requirement enforces that the Euclidean distance
 * between point and section is equal to the specified value.
 */
class PointLineDist final : public Requirement {
    Figures::Point2D*                _p;    ///< pointer to point.
    Figures::Line<Figures::Point2D>* _l;    ///< pointer to line.
    double                           _dist; ///< distance between point and line
public:
    /**
     * @brief Construct a new PointSectionDist requirement.
     * @param p Point.
     * @param l Line.
     * @param dist Fixed distance value between the point and line.
     */
    PointLineDist(Figures::Point2D*                p,
                  Figures::Line<Figures::Point2D>* l,
                  double                           dist);
    /**
      * @brief Build the function for the point section distance constraint.
      *
      * The equation is derived from the general line equation through
      * segment endpoints A(xₐ, yₐ), B(xᵦ, yᵦ):
      *
      * \f[
      * \begin{aligned}
      * Ax + By + C &= 0 \\
      * A &= y_B - y_A \\
      * B &= x_A - x_B \\
      * C &= x_B y_A - x_A y_B
      * \end{aligned}
      * \f]
      *
      * For a point P(x, y), the signed distance to line AB is:
      *
      * \f[
      * F = \frac{A x_P + B y_P + C}{\sqrt{A^2 + B^2}}
      * \f]
      *
      * The requirement enforces:
      *
      * \f[ F - d = 0 \f]
      *
      * where \f$ d \f$ is the fixed distance value.
      *
      * @return Pointer to function object tree.
      */
    ErrorFunction* toFunction() override;
};

/**
 * @brief Requirement: put the point to the section.
 *
 * This requirement enforces that the Euclidean distance
 * between point and section is equal to 0.
 */
class PointOnLine final : public Requirement {
    Figures::Point2D*                _p; ///< pointer to point.
    Figures::Line<Figures::Point2D>* _l; ///< pointer to line.
public:
    /**
     * @brief Construct a new PointOnLine requirement.
     * @param p Point.
     * @param l Line.
     */
    PointOnLine(Figures::Point2D* p, Figures::Line<Figures::Point2D>* l);
    /**
     * @brief Simular to PointLineDist.
     * @return Pointer to function object tree.
     */
    ErrorFunction* toFunction() override;
};
/**
 * @brief Constraint: Fixed distance between two points.
 *
 * This requirement enforces a specific Euclidean distance between two points
 * in 2D space. The constraint maintains the exact distance value regardless
 * of the points' positions.
 */
class PointPointDist final : public Requirement {
    Figures::Point2D* _p1; ///< pointer to first point.
    Figures::Point2D* _p2; ///< pointer to second point.
    double            _dist; ///< distance between two points.

public:
    /**
     * @brief Construct a new Point-Point Distance constraint.
     *
     * @param p1 First point
     * @param p2 Second point
     * @param dist Distance between 2 points
     */
    PointPointDist(Figures::Point2D* p1, Figures::Point2D* p2, double dist);
    /**
     * @brief Convert the constraint to a function for solving.
     *
     * Creates a function tree representing the constraint equation:
     * \f[
     * (x_2 - x_1)^2 + (y_2 - y_1)^2 - d^2 = 0
     * \f]
     * where \f$d\f$ is the required distance.
     *
     * @return ErrorFunction* Pointer to the constraint function
     */
    ErrorFunction* toFunction() override;
};

/**
 * @brief Constraint: Two points must coincide.
 *
 * This requirement enforces that two points have identical coordinates,
 * effectively making them reference the same geometric location.
 */
class PointOnPoint final : public Requirement {
    Figures::Point2D* _p1;
    Figures::Point2D* _p2;
    public:
/** @brief Construct a new PointOnPoint requirement
 *
 *  @param p1 First point
 *  @param p2 second point
 */
    PointOnPoint(Figures::Point2D* p1, Figures::Point2D* p2);
    /**
     * @brief Convert the constraint to a function for solving.
     * This constraint use redefinition. We use two name for
     * one point for more flexability.
     * \f[
     * p_1 == p_2
     * \f]
     * @return ErrorFunction* Pointer to the constraint function
     */
    ErrorFunction* toFunction() override;
};

}
#endif //OURPAINTDCM_HEADERS_REQUIREMENTS_REQUIREMENTS_H