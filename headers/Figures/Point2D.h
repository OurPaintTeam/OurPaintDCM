#ifndef OURPAINTDCM_HEADERS_FIGURES_POINT2D_H
#define OURPAINTDCM_HEADERS_FIGURES_POINT2D_H

namespace OurPaintDCM {
    namespace Figures {
        /**
        * @brief 2D point structure.
        *
        * Represents a point in 2D space with X and Y
        * Basic object for circles, arcs, sections etc.
        */
        struct Point2D {
            double _x; ///< X coordinates
            double _y; ///< Y coordinates
        };
    }
}
#endif //OURPAINTDCM_HEADERS_FIGURES_POINT2D_H
