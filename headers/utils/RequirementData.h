#ifndef HEADERS_UTILS_REQUIREMENTDATA_H
#define HEADERS_UTILS_REQUIREMENTDATA_H
#include "Enums.h"
#include "ID.h"
#include "FigureData.h"
#include "Requirements.h"

#include <utility>
#include <vector>



namespace OurPaintDCM::Utils {
/**
 * @brief Diagnostic description of a geometric requirement.
 * This structure is generated on demand and returned to the user
 * (it is not stored permanently inside the solver).
 * It contains information about the requirement type and the
 * geometric objects it references. */
struct RequirementData {
    Requirements::Requirement* requirement;
    RequirementType            type;
    ID                         id;
    std::vector<FigureData>    objects;
    /**
     * @brief Default constructor. */
    RequirementData() = default;
    /**
     * @brief Construct RequirementData from fields.
     * @param t Requirement type.
     * @param req Unique identifier of the requirement.
     * @param objs List of affected geometric objects. */
    RequirementData(RequirementType t, Requirements::Requirement* req, std::vector<FigureData> objs)
        : requirement(req),
          type(t),
          objects(std::move(objs)),
          id(-11) {}
};
}
#endif //HEADERS_UTILS_REQUIREMENTDATA_H