#ifndef HEADERS_UTILS_REQUIREMENTDATA_H
#define HEADERS_UTILS_REQUIREMENTDATA_H
#include "Enums.h"
#include "ID.h"
#include <vector>

namespace OurPaintDCM::Utils {
/**
 * @brief Diagnostic description of a geometric requirement.
 * This structure is generated on demand and returned to the user
 * (it is not stored permanently inside the solver).
 * It contains information about the requirement type and the
 * geometric objects it references. */
struct RequirementData {
    /// Type of the requirement (e.g., Coincidence, Parallel, Perpendicular).
    RequirementType type;
    /// Unique identifier of the requirement.
    ID id;
    /// IDs of the geometric objects affected by this requirement.
    std::vector<ID> objects;
    /**
     * @brief Default constructor. */
    RequirementData() = default;
    /**
     * @brief Construct RequirementData from fields.
     * @param t Requirement type.
     * @param reqId Unique identifier of the requirement.
     * @param objs List of affected geometric objects. */
    RequirementData(RequirementType t, ID reqId, std::vector<ID> objs)
        : type(t),
          id(reqId),
          objects(std::move(objs)) {
    }
};
}
#endif //HEADERS_UTILS_REQUIREMENTDATA_H