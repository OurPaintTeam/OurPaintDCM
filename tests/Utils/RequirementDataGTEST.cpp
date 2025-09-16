#include <gtest/gtest.h>
#include "RequirementData.h"

using namespace OurPaintDCM::Utils;

TEST(RequirementDataTest, DefaultConstructor) {
    RequirementData req;
    EXPECT_EQ(req.objects.size(), 0);
    EXPECT_EQ(req.id.id, 0);
}

TEST(RequirementDataTest, ParameterizedConstructor) {
    std::vector<ID> objs = {ID(1), ID(2)};
    RequirementData req(RequirementType::ET_LINEINCIRCLE, ID(10), objs);

    EXPECT_EQ(req.type, RequirementType::ET_LINEINCIRCLE);
    EXPECT_EQ(req.id.id, 10);
    ASSERT_EQ(req.objects.size(), 2);
    EXPECT_EQ(req.objects[0].id, 1);
    EXPECT_EQ(req.objects[1].id, 2);
}

TEST(RequirementDataTest, MoveConstructorObjects) {
    std::vector<ID> objs = {ID(5), ID(6)};
    RequirementData req(RequirementType::ET_LINELINEPARALLEL, ID(20), std::move(objs));

    EXPECT_EQ(req.type, RequirementType::ET_LINELINEPARALLEL);
    EXPECT_EQ(req.id.id, 20);
    ASSERT_EQ(req.objects.size(), 2);
    EXPECT_EQ(req.objects[0].id, 5);
    EXPECT_EQ(req.objects[1].id, 6);
    EXPECT_EQ(objs.size(), 0);
}
