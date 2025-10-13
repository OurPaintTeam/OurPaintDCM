#include <gtest/gtest.h>
#include "GlobalRequirementManager.h"

using namespace OurPaintDCM;

TEST(GlobalSolveModeTest, SetModeDoesNotThrow) {
    GlobalRequirementManager mgr;
    EXPECT_NO_THROW(mgr.setMode(Utils::SolveMode::DRAG));
}

TEST(GlobalSolveModeTest, GetModeTest) {
    GlobalRequirementManager mgr;
    mgr.setMode(Utils::SolveMode::DRAG);
    EXPECT_EQ(Utils::SolveMode::DRAG, mgr.getMode());
}
