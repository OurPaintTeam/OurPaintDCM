#ifndef HEADERS_UTILS_REQUIREMENTDATA_H
#define HEADERS_UTILS_REQUIREMENTDATA_H
#include "Enums.h"
#include "ID.h"
#include "FigureData.h"
#include "Requirements.h"

#include <utility>
#include <vector>
#include <cstddef>

namespace OurPaintDCM::Utils {
    /**
     * @brief Diagnostic description of a geometric requirement.
     * This structure is generated on demand and returned to the user
     * (it is not stored permanently inside the solver).
     * It contains information about the requirement type and the
     * geometric objects it references. */
    struct RequirementData {
        Requirements::Requirement *requirement;
        RequirementType type;
        ID id;
        std::vector<FigureData> objects;
        double param;
        /**
         * @brief Default constructor. */
        RequirementData() = default;

        /**
         * @brief Construct RequirementData from fields.
         * @param t Requirement type.
         * @param req Unique identifier of the requirement.
         * @param objs List of affected geometric objects. */
        RequirementData(RequirementType t, Requirements::Requirement *req, std::vector<FigureData> objs)
            : requirement(req),
              type(t),
              objects(std::move(objs)),
              id(-11) {
            if (!req) {
                param = 0;
            } else {
                param = req->getParam();
            }
        }

        RequirementData(const RequirementData &other) = default;
        RequirementData(RequirementData&& other) = default;
        bool operator==(const RequirementData &other) const {
            return type == other.type &&
                   id == other.id &&
                   objects == other.objects &&
                   param == other.param;
        }
    };
}

namespace std {
    template<>
    struct hash<OurPaintDCM::Utils::RequirementData> {
        size_t operator()(const OurPaintDCM::Utils::RequirementData &req) const noexcept {
            using namespace OurPaintDCM::Utils;

            size_t seed = 0;
            auto hash_combine = [&](size_t value) {
                seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            };

            // type
            hash_combine(std::hash<int>{}(static_cast<int>(req.type)));

            // id
            hash_combine(std::hash<ID>{}(req.id));

            // objects
            for (const FigureData &fig: req.objects) {
                hash_combine(std::hash<unsigned long long>{}(fig.id.id));
            }
            return seed;
        }
    };
}
#endif //HEADERS_UTILS_REQUIREMENTDATA_H
