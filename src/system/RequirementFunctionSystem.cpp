#include "RequirementFunctionSystem.h"

OurPaintDCM::Utils::SystemStatus OurPaintDCM::System::RequirementFunctionSystem::diagnose() const {
    switch (_math.diagnose()) {
        case Math::ConstraintStatus::EMPTY: return Utils::SystemStatus::EMPTY;
        case Math::ConstraintStatus::WELL_CONSTRAINED: return Utils::SystemStatus::WELL_CONSTRAINED;
        case Math::ConstraintStatus::UNDER_CONSTRAINED: return Utils::SystemStatus::UNDER_CONSTRAINED;
        case Math::ConstraintStatus::OVER_CONSTRAINED: return Utils::SystemStatus::OVER_CONSTRAINED;
        case Math::ConstraintStatus::SINGULAR_SYSTEM: return Utils::SystemStatus::SINGULAR_SYSTEM;
        default: return Utils::SystemStatus::UNKNOWN;
    }
}
