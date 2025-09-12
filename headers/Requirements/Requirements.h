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
    Figures::Point2D*                             _p;    ///< pointer to point.
    Figures::Line<Figures::Point2D>* _l;    ///< pointer to line.
    double                                                     _dist; ///< distance between point and line
public:
    /**
     * @brief Construct a new PointSectionDist requirement.
     * @param p First point.
     * @param l Second point.
     * @param dist Fixed distance value between the point and line.
     */
    PointLineDist(Figures::Point2D*                             p,
                     Figures::Line<Figures::Point2D>* l,
                     double                                                     dist);
    /**
     * @brief Build the function for the point section distance constraint.
     *
     * The equation is derived from the general line equation through
     * segment endpoints A(xA, yA), B(xB, yB):
     * \f[
     * Ax + By + C = 0, \quad
     * A = y_B - y_A, \quad B = x_A - x_B, \quad C = x_B y_A - x_A y_B
     * \f]
     *
     * For a point P(x, y), the signed distance to line AB is:
     * \f[
     * F = \frac{A x_P + B y_P + C}{\sqrt{A^2 + B^2}}
     * \f]
     *
     * The requirement enforces:
     * \f[
     * F - d = 0
     * \f]
     * where d is the fixed distance value.
     *
     * @return Pointer to function object tree.
     */
    ErrorFunction* toFunction() override;
};
}
#endif //OURPAINTDCM_HEADERS_REQUIREMENTS_REQUIREMENTS_H