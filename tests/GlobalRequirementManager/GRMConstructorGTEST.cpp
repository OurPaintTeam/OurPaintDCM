#include <gtest/gtest.h>
#include "GlobalRequirementManager.h"

using namespace OurPaintDCM;

TEST(GlobalRequirementManagerTest, DefaultConstructorInitializesEmpty) {
    EXPECT_NO_THROW({GlobalRequirementManager mgr;});
}

// Additional check: destruction does not throw
TEST(GlobalRequirementManagerTest, DestructorDoesNotThrow) {
    EXPECT_NO_THROW({
        auto* mgr = new GlobalRequirementManager();
        delete mgr; // ensure destructor runs cleanly
    });
}
