#include <gtest/gtest.h>
#include "Point2D.h"
#include "math.h"

using namespace OurPaintDCM::Figures;

class Point2DTest : public ::testing::Test {
protected:
    Point2D p;

    void SetUp() override {
        p._x = 0.0;
        p._y = 0.0;
    }
};

TEST_F(Point2DTest, DefaultCoordinates) {
    EXPECT_DOUBLE_EQ(p._x, 0.0);
    EXPECT_DOUBLE_EQ(p._y, 0.0);
}

TEST_F(Point2DTest, SetCoordinates) {
    p._x = 5.5;
    p._y = -3.2;
    EXPECT_DOUBLE_EQ(p._x, 5.5);
    EXPECT_DOUBLE_EQ(p._y, -3.2);
}
TEST_F(Point2DTest, DistanceToAnotherPoint) {
    Point2D p2;
    p2._x = 3.0;
    p2._y = 4.0;

    double dx = p2._x - p._x;
    double dy = p2._y - p._y;
    double distance = std::sqrt(dx * dx + dy * dy);

    EXPECT_DOUBLE_EQ(distance, 5.0);
}