//
// Created by Ardrass on Вт, 16.09.2025.
//

#ifndef HEADERS_UTILS_FIGUREDATA_H
#define HEADERS_UTILS_FIGUREDATA_H

#include "ID.h"
#include "Enums.h"
#include <string>
#include <vector>

namespace OurPaintDCM::Utils {

/**
 * @brief Diagnostic description of a geometric figure.
 *
 * This structure is generated on demand and returned to the user
 * (it is not stored permanently inside the solver).
 *
 * It contains information about the figure type, its unique ID,
 * and IDs of points or sub-objects that define it.
 */
struct FigureData {
    /// Type of the figure (e.g., Line, Circle, Arc).
    FigureType type;

    /// Unique identifier of the figure.
    ID id;
    /// Parameters of figures(e.g. start point, end point, center).
    std::vector<double> params;

    /// IDs of geometric primitives that define this figure (e.g., points).
    std::vector<ID> subObjects;
    /**
     * @brief Default constructor.
     */
    FigureData() = default;

    /**
     * @brief Construct FigureData from fields.
     *
     * @param t Figure type.
     * @param figId Unique identifier of the figure.
     * @param params List of params that define the figure.
     */
    FigureData(FigureType t, ID figId, std::vector<double> params, std::vector<ID> subObjects)
        : type(t),
          id(figId),
          params(std::move(params)),
          subObjects(std::move(subObjects)) {
    }
};

}

#endif //HEADERS_UTILS_FIGUREDATA_H