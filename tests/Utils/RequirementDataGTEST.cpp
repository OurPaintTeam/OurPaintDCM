#include <gtest/gtest.h>
#include "RequirementData.h"

using namespace OurPaintDCM::Utils;

TEST(RequirementDataTest, DefaultConstructor) {
    RequirementData req;
    EXPECT_EQ(req.objects.size(), 0);
    EXPECT_EQ(req.id.id, 0);
}

TEST(RequirementDataTest, ParameterizedConstructor) {
    std::vector<FigureData> objs = {FigureData(FigureType::ET_POINT2D, ID(1), std::vector<double>(), std::vector<ID>()), FigureData(FigureType::ET_POINT2D, ID(2), std::vector<double>(), std::vector<ID>())};
    RequirementData req(RequirementType::ET_LINEINCIRCLE, nullptr, objs);

    EXPECT_EQ(req.type, RequirementType::ET_LINEINCIRCLE);
    EXPECT_EQ(req.id, -11);
    ASSERT_EQ(req.objects.size(), 2);
    EXPECT_EQ(req.objects[0].id, 1);
    EXPECT_EQ(req.objects[1].id, 2);
}

TEST(RequirementDataTest, MoveConstructorObjects) {
    std::vector<FigureData> objs = {FigureData(FigureType::ET_POINT2D, ID(5), std::vector<double>(), std::vector<ID>()), FigureData(FigureType::ET_POINT2D, ID(6), std::vector<double>(), std::vector<ID>())};
    RequirementData req(RequirementType::ET_LINELINEPARALLEL, nullptr, std::move(objs));

    EXPECT_EQ(req.type, RequirementType::ET_LINELINEPARALLEL);
    EXPECT_EQ(req.id.id, -11);
    ASSERT_EQ(req.objects.size(), 2);
    EXPECT_EQ(req.objects[0].id, 5);
    EXPECT_EQ(req.objects[1].id, 6);
    EXPECT_EQ(objs.size(), 0);
}
