#include "utils/FigureData.h"
#include <gtest/gtest.h>

using namespace OurPaintDCM::Utils;

TEST(FigureDataTest, DefaultConstructor) {
    FigureData fig;
    EXPECT_EQ(fig.params.size(), 0);
    EXPECT_EQ(fig.subObjects.size(), 0);
    EXPECT_EQ(fig.id.id, 0);
}

TEST(FigureDataTest, ParameterizedConstructor) {
    std::vector<double> params = {0.0, 1.0, 2.0};
    std::vector<ID> subs = {ID(10), ID(20)};
    FigureData fig(FigureType::ET_LINE, ID(5), params, subs);

    EXPECT_EQ(fig.type, FigureType::ET_LINE);
    EXPECT_EQ(fig.id.id, 5);
    ASSERT_EQ(fig.params.size(), 3);
    EXPECT_DOUBLE_EQ(fig.params[0], 0.0);
    EXPECT_DOUBLE_EQ(fig.params[1], 1.0);
    EXPECT_DOUBLE_EQ(fig.params[2], 2.0);
    ASSERT_EQ(fig.subObjects.size(), 2);
    EXPECT_EQ(fig.subObjects[0].id, 10);
    EXPECT_EQ(fig.subObjects[1].id, 20);
}

TEST(FigureDataTest, MoveConstructorParamsAndSubObjects) {
    std::vector<double> params = {3.0, 4.0};
    std::vector<ID> subs = {ID(100)};
    FigureData fig(FigureType::ET_CIRCLE, ID(7), std::move(params), std::move(subs));

    EXPECT_EQ(fig.type, FigureType::ET_CIRCLE);
    EXPECT_EQ(fig.id.id, 7);
    ASSERT_EQ(fig.params.size(), 2);
    EXPECT_DOUBLE_EQ(fig.params[0], 3.0);
    EXPECT_DOUBLE_EQ(fig.params[1], 4.0);
    ASSERT_EQ(fig.subObjects.size(), 1);
    EXPECT_EQ(fig.subObjects[0].id, 100);
    EXPECT_EQ(params.size(), 0);
    EXPECT_EQ(subs.size(), 0);
}
