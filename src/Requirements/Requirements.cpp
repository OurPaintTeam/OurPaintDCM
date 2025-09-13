#include "Requirements.h"

namespace OurPaintDCM::Requirements {
// ----------------------------------------------------------------
    PointLineDist::PointLineDist(Figures::Point2D* p, Figures::Line<Figures::Point2D>* l, double dist)
        : Requirement(RequirementType::ET_POINTLINEDIST),
          _p(p),
          _l(l),
          _dist(dist) {
    }

    ErrorFunction* PointLineDist::toFunction() {
        std::vector<Variable*> x;
        auto*                  p_x      = new Variable(_p->ptrX());
        auto*                  p_y      = new Variable(_p->ptrY());
        auto*                  l_startx = new Variable(_l->p1->ptrX());
        auto*                  l_starty = new Variable(_l->p1->ptrY());
        auto*                  l_endx   = new Variable(_l->p2->ptrX());
        auto*                  l_endy   = new Variable(_l->p2->ptrY());
        x.push_back(p_x);
        x.push_back(p_y);
        x.push_back(l_startx);
        x.push_back(l_starty);
        x.push_back(l_endx);
        x.push_back(l_endy);
        return new PointSectionDistanceError(x, _dist);
    }

    //-----------------------------------------------------------------
    PointOnLine::PointOnLine(Figures::Point2D* p, Figures::Line<Figures::Point2D>* l)
        : Requirement(RequirementType::ET_POINTONLINE),
          _p(p),
          _l(l){}
    ErrorFunction* PointOnLine::toFunction() {
        std::vector<Variable*> x;
        auto*                  p_x      = new Variable(_p->ptrX());
        auto*                  p_y      = new Variable(_p->ptrY());
        auto*                  l_startx = new Variable(_l->p1->ptrX());
        auto*                  l_starty = new Variable(_l->p1->ptrY());
        auto*                  l_endx   = new Variable(_l->p2->ptrX());
        auto*                  l_endy   = new Variable(_l->p2->ptrY());
        x.push_back(p_x);
        x.push_back(p_y);
        x.push_back(l_startx);
        x.push_back(l_starty);
        x.push_back(l_endx);
        x.push_back(l_endy);
        return new PointOnSectionError(x);
    }

}