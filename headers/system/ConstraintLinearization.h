#ifndef OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTLINEARIZATION_H
#define OURPAINTDCM_HEADERS_SYSTEM_CONSTRAINTLINEARIZATION_H

#include "RequirementFunction.h"
#include "SparseMatrix.h"
#include <vector>

namespace OurPaintDCM::System {

struct ConstraintEquation {
    std::size_t functionIndex;
    std::optional<Utils::ID> requirementId;
    double weight;
};

/** Owned numerical snapshot in the original function/coordinate order.
 * Variables are borrowed coordinate identities; analysis must not dereference them.
 * Rows include disabled bindings and implicit constraints without a requirement ID.
 * Function indices are local to this snapshot; rebuild after topology changes.
 */
struct ConstraintLinearization {
    SparseMatrix<> jacobian;
    Matrix<> residuals;
    std::vector<VAR> variables;
    std::vector<ConstraintEquation> equations;
    bool defined = false;
};

} // namespace OurPaintDCM::System

#endif
