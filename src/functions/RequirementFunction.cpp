#include "functions/RequirementFunction.h"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace {

constexpr double kGeometryEpsilon = 1e-12;

std::unordered_map<VAR, double> makeZeroGradient(const std::vector<VAR>& vars) {
    std::unordered_map<VAR, double> grad;
    for (VAR var : vars) {
        grad[var] = 0.0;
    }
    return grad;
}

double pointLineSignedDistance(const std::vector<VAR>& vars) {
    const double px = *vars[0];
    const double py = *vars[1];
    const double x1 = *vars[2];
    const double y1 = *vars[3];
    const double x2 = *vars[4];
    const double y2 = *vars[5];

    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double lineLen = std::sqrt(dx * dx + dy * dy);
    if (lineLen < kGeometryEpsilon) {
        return 0.0;
    }

    const double wx = px - x1;
    const double wy = py - y1;
    const double cross = wx * dy - wy * dx;
    return cross / lineLen;
}

std::unordered_map<VAR, double> pointLineSignedDistanceGradient(const std::vector<VAR>& vars) {
    const double px = *vars[0];
    const double py = *vars[1];
    const double x1 = *vars[2];
    const double y1 = *vars[3];
    const double x2 = *vars[4];
    const double y2 = *vars[5];

    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double lineLen = std::sqrt(dx * dx + dy * dy);
    if (lineLen < kGeometryEpsilon) {
        return makeZeroGradient(vars);
    }

    const double wx = px - x1;
    const double wy = py - y1;
    const double cross = wx * dy - wy * dx;
    const double lineLen3 = lineLen * lineLen * lineLen;

    // The same point may occupy multiple argument positions, so sum their derivatives.
    std::unordered_map<VAR, double> grad;
    grad[vars[0]] += dy / lineLen;
    grad[vars[1]] += -dx / lineLen;
    grad[vars[2]] += (py - y2) / lineLen + cross * dx / lineLen3;
    grad[vars[3]] += (x2 - px) / lineLen + cross * dy / lineLen3;
    grad[vars[4]] += (y1 - py) / lineLen - cross * dx / lineLen3;
    grad[vars[5]] += (px - x1) / lineLen - cross * dy / lineLen3;
    return grad;
}

} // namespace

//PointLineDistanceFunction Requirement
OurPaintDCM::Function::PointLineDistanceFunction::PointLineDistanceFunction(
    const std::vector<VAR> &vars, double dist) : RequirementFunction(
    Utils::RequirementType::ET_POINTLINEDIST, vars) {
    if (vars.size() != 6) {
        throw std::invalid_argument("This function must have 6 variables");
    }
    Utils::validateRequirementParameter(Utils::RequirementType::ET_POINTLINEDIST, dist);
    _distance = dist;
}

double OurPaintDCM::Function::PointLineDistanceFunction::evaluate() const {
    return pointLineSignedDistance(_vars) - _distance;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::PointLineDistanceFunction::gradient() const {
    return pointLineSignedDistanceGradient(_vars);
}

size_t OurPaintDCM::Function::PointLineDistanceFunction::getVarCount() const {
    return 6;
}

//PointOnLineFunction Requirement

OurPaintDCM::Function::PointOnLineFunction::PointOnLineFunction(const std::vector<VAR> &vars) : RequirementFunction(
    Utils::RequirementType::ET_POINTONLINE, vars) {
    if (vars.size() != 6) {
        throw std::invalid_argument("This function must have 6 variables");
    }
}

double OurPaintDCM::Function::PointOnLineFunction::evaluate() const {
    return pointLineSignedDistance(_vars);
}

std::unordered_map<VAR, double> OurPaintDCM::Function::PointOnLineFunction::gradient() const {
    return pointLineSignedDistanceGradient(_vars);
}

size_t OurPaintDCM::Function::PointOnLineFunction::getVarCount() const {
    return 6;
}

// PointPointDistanceFunction Requirement
OurPaintDCM::Function::PointPointDistanceFunction::PointPointDistanceFunction(
    const std::vector<VAR> &vars, double dist) : RequirementFunction(
    Utils::RequirementType::ET_POINTPOINTDIST, vars) {
    if (vars.size() != 4) {
        throw std::invalid_argument("This function must have 4 variables");
    }
    Utils::validateRequirementParameter(Utils::RequirementType::ET_POINTPOINTDIST, dist);
    _distance = dist;
}

double OurPaintDCM::Function::PointPointDistanceFunction::evaluate() const {
    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = std::sqrt(dx * dx + dy * dy);

    return dist - _distance;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::PointPointDistanceFunction::gradient() const {
    std::unordered_map<VAR, double> grad;

    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1e-10) {
        grad[_vars[0]] = 0.0;
        grad[_vars[1]] = 0.0;
        grad[_vars[2]] = 0.0;
        grad[_vars[3]] = 0.0;
        return grad;
    }

    // df/dx1 = -(x2 - x1)/dist
    grad[_vars[0]] += -dx / dist;
    // df/dy1 = -(y2 - y1)/dist
    grad[_vars[1]] += -dy / dist;
    // df/dx2 =  (x2 - x1)/dist
    grad[_vars[2]] += dx / dist;
    // df/dy2 =  (y2 - y1)/dist
    grad[_vars[3]] += dy / dist;

    return grad;
}
size_t OurPaintDCM::Function::PointPointDistanceFunction::getVarCount() const {
    return 4;
}
//PointOnPointFunction Requirement
OurPaintDCM::Function::PointOnPointFunction::PointOnPointFunction(const std::vector<VAR> &vars) : RequirementFunction(
    Utils::RequirementType::ET_POINTONPOINT, vars) {
    if (vars.size() != 4) {
        throw std::invalid_argument("This function must have 4 variables");
    }
}

double OurPaintDCM::Function::PointOnPointFunction::evaluate() const {
    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = std::sqrt(dx * dx + dy * dy);

    return dist;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::PointOnPointFunction::gradient() const {
    std::unordered_map<VAR, double> grad;
    double dx = *_vars[2] - *_vars[0];
    double dy = *_vars[3] - *_vars[1];
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1e-10) {
        grad[_vars[0]] = 0.0;
        grad[_vars[1]] = 0.0;
        grad[_vars[2]] = 0.0;
        grad[_vars[3]] = 0.0;
        return grad;
    }

    // df/dx1 = -(x2 - x1)/dist
    grad[_vars[0]] += -dx / dist;
    // df/dy1 = -(y2 - y1)/dist
    grad[_vars[1]] += -dy / dist;
    // df/dx2 =  (x2 - x1)/dist
    grad[_vars[2]] += dx / dist;
    // df/dy2 =  (y2 - y1)/dist
    grad[_vars[3]] += dy / dist;

    return grad;
}

size_t OurPaintDCM::Function::PointOnPointFunction::getVarCount() const {
    return 4;
}

// LineCircleDistanceFunction Requirement

OurPaintDCM::Function::LineCircleDistanceFunction::LineCircleDistanceFunction(
    const std::vector<VAR> &vars, double dist) : RequirementFunction(
    Utils::RequirementType::ET_LINECIRCLEDIST, vars) {
    if (vars.size() != 7) {
        throw std::invalid_argument("This function must have 7 variables");
    }
    Utils::validateRequirementParameter(Utils::RequirementType::ET_LINECIRCLEDIST, dist);
    _distance = dist;
}

double OurPaintDCM::Function::LineCircleDistanceFunction::evaluate() const {
    const double dx = *_vars[2] - *_vars[0];
    const double dy = *_vars[3] - *_vars[1];
    const double lengthSquared = dx * dx + dy * dy;
    const double t = lengthSquared > kGeometryEpsilon * kGeometryEpsilon
        ? std::clamp(((*_vars[4] - *_vars[0]) * dx + (*_vars[5] - *_vars[1]) * dy)
                         / lengthSquared, 0.0, 1.0)
        : 0.0;
    return std::hypot(*_vars[0] + t * dx - *_vars[4],
                      *_vars[1] + t * dy - *_vars[5]) - *_vars[6] - _distance;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::LineCircleDistanceFunction::gradient() const {
    const double dx = *_vars[2] - *_vars[0];
    const double dy = *_vars[3] - *_vars[1];
    const double lengthSquared = dx * dx + dy * dy;
    const double t = lengthSquared > kGeometryEpsilon * kGeometryEpsilon
        ? std::clamp(((*_vars[4] - *_vars[0]) * dx + (*_vars[5] - *_vars[1]) * dy)
                         / lengthSquared, 0.0, 1.0)
        : 0.0;
    const double diffX = *_vars[0] + t * dx - *_vars[4];
    const double diffY = *_vars[1] + t * dy - *_vars[5];
    const double distance = std::hypot(diffX, diffY);
    auto grad = makeZeroGradient(_vars);
    // At zero distance the position derivative is undefined; use the zero subgradient.
    // The radius derivative is still -1, including at a collapsed segment.
    if (distance > kGeometryEpsilon) {
        const double nx = diffX / distance;
        const double ny = diffY / distance;
        // For an interior projection the normal is perpendicular to the segment,
        // so derivatives of t cancel. At either endpoint t is clamped and constant.
        grad[_vars[0]] += (1.0 - t) * nx;
        grad[_vars[1]] += (1.0 - t) * ny;
        grad[_vars[2]] += t * nx;
        grad[_vars[3]] += t * ny;
        grad[_vars[4]] -= nx;
        grad[_vars[5]] -= ny;
    }
    grad[_vars[6]] -= 1.0;
    return grad;
}

size_t OurPaintDCM::Function::LineCircleDistanceFunction::getVarCount() const {
    return 7;
}

OurPaintDCM::Function::PointOnCircleFunction::PointOnCircleFunction(const std::vector<VAR>& vars)
    : RequirementFunction(Utils::RequirementType::ET_LINEONCIRCLE, vars) {
    if (vars.size() != 5) {
        throw std::invalid_argument("This function must have 5 variables");
    }
}

double OurPaintDCM::Function::PointOnCircleFunction::evaluate() const {
    return std::hypot(*_vars[0] - *_vars[2], *_vars[1] - *_vars[3]) - *_vars[4];
}

std::unordered_map<VAR, double> OurPaintDCM::Function::PointOnCircleFunction::gradient() const {
    const double dx = *_vars[0] - *_vars[2];
    const double dy = *_vars[1] - *_vars[3];
    const double distance = std::hypot(dx, dy);
    auto grad = makeZeroGradient(_vars);
    if (distance > kGeometryEpsilon) {
        grad[_vars[0]] += dx / distance;
        grad[_vars[1]] += dy / distance;
        grad[_vars[2]] -= dx / distance;
        grad[_vars[3]] -= dy / distance;
    }
    grad[_vars[4]] -= 1.0;
    return grad;
}

size_t OurPaintDCM::Function::PointOnCircleFunction::getVarCount() const {
    return 5;
}

//LineOnCircleFunction Requirement
OurPaintDCM::Function::LineOnCircleFunction::LineOnCircleFunction(
    const std::vector<VAR>& vars) : RequirementFunction(Utils::RequirementType::ET_LINEONCIRCLE, vars) {
    if (vars.size() != 7) {
        throw std::invalid_argument("This function must have 7 variables");
    }
}

double OurPaintDCM::Function::LineOnCircleFunction::evaluate() const {
    const PointOnCircleFunction first({_vars[0], _vars[1], _vars[4], _vars[5], _vars[6]});
    const PointOnCircleFunction second({_vars[2], _vars[3], _vars[4], _vars[5], _vars[6]});
    return std::hypot(first.evaluate(), second.evaluate());
}

std::unordered_map<VAR, double> OurPaintDCM::Function::LineOnCircleFunction::gradient() const {
    const PointOnCircleFunction first({_vars[0], _vars[1], _vars[4], _vars[5], _vars[6]});
    const PointOnCircleFunction second({_vars[2], _vars[3], _vars[4], _vars[5], _vars[6]});
    const double firstResidual = first.evaluate();
    const double secondResidual = second.evaluate();
    const double norm = std::hypot(firstResidual, secondResidual);
    auto grad = makeZeroGradient(_vars);
    if (norm > kGeometryEpsilon) {
        for (const auto& [var, derivative] : first.gradient()) {
            grad[var] += firstResidual / norm * derivative;
        }
        for (const auto& [var, derivative] : second.gradient()) {
            grad[var] += secondResidual / norm * derivative;
        }
    }
    return grad;
}

size_t OurPaintDCM::Function::LineOnCircleFunction::getVarCount() const {
    return 7;
}
//LineLineParallelFunction Requirement
OurPaintDCM::Function::LineLineParallelFunction::LineLineParallelFunction(
    const std::vector<VAR> &vars) : RequirementFunction(
    Utils::RequirementType::ET_LINELINEPARALLEL, vars) {
    if (vars.size() != 8) {
        throw std::invalid_argument("This function must have 8 variables");
    }
}

double OurPaintDCM::Function::LineLineParallelFunction::evaluate() const {
    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];
    double x3 = *_vars[4];
    double y3 = *_vars[5];
    double x4 = *_vars[6];
    double y4 = *_vars[7];

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x4 - x3;
    double dy2 = y4 - y3;

    double cross = dx1 * dy2 - dy1 * dx2;

    return cross;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::LineLineParallelFunction::gradient() const {
    std::unordered_map<VAR, double> grad;

    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];
    double x3 = *_vars[4];
    double y3 = *_vars[5];
    double x4 = *_vars[6];
    double y4 = *_vars[7];

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x4 - x3;
    double dy2 = y4 - y3;

    grad[_vars[0]] += -dy2;
    grad[_vars[1]] += dx2;
    grad[_vars[2]] += dy2;
    grad[_vars[3]] += -dx2;

    grad[_vars[4]] += dy1;
    grad[_vars[5]] += -dx1;
    grad[_vars[6]] += -dy1;
    grad[_vars[7]] += dx1;

    return grad;
}

size_t OurPaintDCM::Function::LineLineParallelFunction::getVarCount() const {
    return 8;
}

// LineLinePerpendicularFunction Requirement
OurPaintDCM::Function::LineLinePerpendicularFunction::LineLinePerpendicularFunction(
    const std::vector<VAR> &vars) : RequirementFunction(
    Utils::RequirementType::ET_LINELINEPERPENDICULAR, vars) {
    if (vars.size() != 8) {
        throw std::invalid_argument("This function must have 8 variables");
    }
}

double OurPaintDCM::Function::LineLinePerpendicularFunction::evaluate() const {
    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];
    double x3 = *_vars[4];
    double y3 = *_vars[5];
    double x4 = *_vars[6];
    double y4 = *_vars[7];

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x4 - x3;
    double dy2 = y4 - y3;

    return dx1 * dx2 + dy1 * dy2;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::LineLinePerpendicularFunction::gradient() const {
    std::unordered_map<VAR, double> grad;

    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];
    double x3 = *_vars[4];
    double y3 = *_vars[5];
    double x4 = *_vars[6];
    double y4 = *_vars[7];

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x4 - x3;
    double dy2 = y4 - y3;

    grad[_vars[0]] += -dx2; // A1x
    grad[_vars[1]] += -dy2; // A1y
    grad[_vars[2]] += dx2; // A2x
    grad[_vars[3]] += dy2; // A2y

    grad[_vars[4]] += -dx1; // B1x
    grad[_vars[5]] += -dy1; // B1y
    grad[_vars[6]] += dx1; // B2x
    grad[_vars[7]] += dy1; // B2y

    return grad;
}

size_t OurPaintDCM::Function::LineLinePerpendicularFunction::getVarCount() const {
    return 8;
}

// LineLineAngleFunction Requirement
OurPaintDCM::Function::LineLineAngleFunction::LineLineAngleFunction(const std::vector<VAR> &vars,
                                                                    double angle) : RequirementFunction(
    Utils::RequirementType::ET_LINELINEANGLE, vars), _angle(angle) {
    Utils::validateRequirementParameter(Utils::RequirementType::ET_LINELINEANGLE, angle);
    if (vars.size() != 8) {
        throw std::invalid_argument("This function must have 8 variables");
    }
}

double OurPaintDCM::Function::LineLineAngleFunction::evaluate() const {
    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];
    double x3 = *_vars[4];
    double y3 = *_vars[5];
    double x4 = *_vars[6];
    double y4 = *_vars[7];

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x4 - x3;
    double dy2 = y4 - y3;

    double dot = dx1 * dx2 + dy1 * dy2;
    double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    if (len1 < 1e-10 || len2 < 1e-10) return 0.0;

    double cos_theta = dot / (len1 * len2);
    return cos_theta - std::cos(_angle);
}

std::unordered_map<VAR, double> OurPaintDCM::Function::LineLineAngleFunction::gradient() const {
    std::unordered_map<VAR, double> grad;

    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];
    double x3 = *_vars[4];
    double y3 = *_vars[5];
    double x4 = *_vars[6];
    double y4 = *_vars[7];

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x4 - x3;
    double dy2 = y4 - y3;

    double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    if (len1 < 1e-10 || len2 < 1e-10) {
        for (VAR v: _vars) grad[v] = 0.0;
        return grad;
    }

    double dot = dx1 * dx2 + dy1 * dy2;
    double len1_3 = len1 * len1 * len1;
    double len2_3 = len2 * len2 * len2;

    const double dCosDdx1 = dx2 / (len1 * len2) - dx1 * dot / (len1_3 * len2);
    const double dCosDdy1 = dy2 / (len1 * len2) - dy1 * dot / (len1_3 * len2);
    const double dCosDdx2 = dx1 / (len1 * len2) - dx2 * dot / (len1 * len2_3);
    const double dCosDdy2 = dy1 / (len1 * len2) - dy2 * dot / (len1 * len2_3);

    grad[_vars[0]] += -dCosDdx1;
    grad[_vars[1]] += -dCosDdy1;
    grad[_vars[2]] += dCosDdx1;
    grad[_vars[3]] += dCosDdy1;
    grad[_vars[4]] += -dCosDdx2;
    grad[_vars[5]] += -dCosDdy2;
    grad[_vars[6]] += dCosDdx2;
    grad[_vars[7]] += dCosDdy2;

    return grad;
}
size_t OurPaintDCM::Function::LineLineAngleFunction::getVarCount() const {
    return 8;
}
// VerticalFunction Requirement
OurPaintDCM::Function::VerticalFunction::VerticalFunction(const std::vector<VAR> &vars) : RequirementFunction(
    Utils::RequirementType::ET_VERTICAL, vars) {
    if (vars.size() != 4) {
        throw std::invalid_argument("This function must have 4 variables");
    }
}

double OurPaintDCM::Function::VerticalFunction::evaluate() const {
    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double len = std::sqrt(dx * dx + dy * dy); // normalize

    if (len < 1e-10)
        return 0.0;

    return dx / len;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::VerticalFunction::gradient() const {
    std::unordered_map<VAR, double> grad;

    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double len2 = dx * dx + dy * dy;
    double len = std::sqrt(len2);

    if (len < 1e-10) {
        for (auto v: _vars) grad[v] = 0.0;
        return grad;
    }

    double len3 = len2 * len;

    grad[_vars[0]] += -1.0 / len + dx * dx / len3; // df/dx1
    grad[_vars[1]] += dx * dy / len3; // df/dy1
    grad[_vars[2]] += 1.0 / len - dx * dx / len3; // df/dx2
    grad[_vars[3]] += -dx * dy / len3; // df/dy2

    return grad;
}

size_t OurPaintDCM::Function::VerticalFunction::getVarCount() const {
    return 4;
}

// HorizontalFunction Requirement
OurPaintDCM::Function::HorizontalFunction::HorizontalFunction(const std::vector<VAR> &vars) : RequirementFunction(
    Utils::RequirementType::ET_HORIZONTAL, vars) {
    if (vars.size() != 4) {
        throw std::invalid_argument("This function must have 4 variables");
    }
}

double OurPaintDCM::Function::HorizontalFunction::evaluate() const {
    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double len = std::sqrt(dx * dx + dy * dy); // normalize

    if (len < 1e-10)
        return 0.0;

    return dy / len;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::HorizontalFunction::gradient() const {
    std::unordered_map<VAR, double> grad;

    double x1 = *_vars[0];
    double y1 = *_vars[1];
    double x2 = *_vars[2];
    double y2 = *_vars[3];

    double dx = x2 - x1;
    double dy = y2 - y1;
    double len2 = dx * dx + dy * dy;
    double len = std::sqrt(len2);

    if (len < 1e-10) {
        for (auto v: _vars) grad[v] = 0.0;
        return grad;
    }

    double len3 = len2 * len;

    // f = dy / len
    grad[_vars[0]] += dx * dy / len3; // df/dx1
    grad[_vars[1]] += -1.0 / len + dy * dy / len3; // df/dy1
    grad[_vars[2]] += -dx * dy / len3; // df/dx2
    grad[_vars[3]] += 1.0 / len - dy * dy / len3; // df/dy2

    return grad;
}


size_t OurPaintDCM::Function::HorizontalFunction::getVarCount() const {
    return 4;
}

// ArcCenterOnPerpendicularFunction Requirement
OurPaintDCM::Function::ArcCenterOnPerpendicularFunction::ArcCenterOnPerpendicularFunction(
    const std::vector<VAR> &vars) : RequirementFunction(
    Utils::RequirementType::ET_ARCCENTERONPERPENDICULAR, vars) {
    if (vars.size() != 6) {
        throw std::invalid_argument("This function must have 6 variables");
    }
}

double OurPaintDCM::Function::ArcCenterOnPerpendicularFunction::evaluate() const {
    double Ax = *_vars[0];
    double Ay = *_vars[1];
    double Bx = *_vars[2];
    double By = *_vars[3];
    double Cx = *_vars[4];
    double Cy = *_vars[5];

    // midpoint M
    double Mx = 0.5 * (Ax + Bx);
    double My = 0.5 * (Ay + By);

    // AB vector
    double dx = Bx - Ax;
    double dy = By - Ay;

    // MC vector
    double mx = Cx - Mx;
    double my = Cy - My;

    // Perpendicular condition: (AB · MC) = 0
    double dot = dx * mx + dy * my;

    return dot;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::ArcCenterOnPerpendicularFunction::gradient() const {
    std::unordered_map<VAR, double> grad;

    double Ax = *_vars[0];
    double Ay = *_vars[1];
    double Bx = *_vars[2];
    double By = *_vars[3];
    double Cx = *_vars[4];
    double Cy = *_vars[5];

    double dx = Bx - Ax;
    double dy = By - Ay;
    double mx = Cx - 0.5 * (Ax + Bx);
    double my = Cy - 0.5 * (Ay + By);

    grad[_vars[0]] += -mx - 0.5 * dx; // df/dAx
    grad[_vars[1]] += -my - 0.5 * dy; // df/dAy
    grad[_vars[2]] += mx - 0.5 * dx; // df/dBx
    grad[_vars[3]] += my - 0.5 * dy; // df/dBy
    grad[_vars[4]] += dx; // df/dCx
    grad[_vars[5]] += dy; // df/dCy

    return grad;
}

size_t OurPaintDCM::Function::ArcCenterOnPerpendicularFunction::getVarCount() const {
    return 6;
}

OurPaintDCM::Function::FixCoordinateFunction::FixCoordinateFunction(
    Utils::RequirementType type, const std::vector<VAR>& vars, double target)
    : RequirementFunction(type, vars), _target(target) {
    Utils::requireFinite(target);
    if (vars.size() != 1) {
        throw std::invalid_argument("This function must have 1 variable");
    }
}

double OurPaintDCM::Function::FixCoordinateFunction::evaluate() const {
    return *_vars[0] - _target;
}

std::unordered_map<VAR, double> OurPaintDCM::Function::FixCoordinateFunction::gradient() const {
    return {{_vars[0], 1.0}};
}

size_t OurPaintDCM::Function::FixCoordinateFunction::getVarCount() const {
    return 1;
}

bool OurPaintDCM::Function::FixCoordinateFunction::tryGetAssignment(VAR& var, double& value) const {
    var = _vars[0];
    value = _target;
    return true;
}
