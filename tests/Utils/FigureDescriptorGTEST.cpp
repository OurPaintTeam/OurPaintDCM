#include <gtest/gtest.h>
#include "FigureDescriptor.h"

using namespace OurPaintDCM::Utils;

TEST(FigureDescriptorTest, DefaultConstructor) {
    FigureDescriptor desc;
    EXPECT_TRUE(desc.pointIds.empty());
    EXPECT_FALSE(desc.x.has_value());
    EXPECT_FALSE(desc.y.has_value());
    EXPECT_FALSE(desc.radius.has_value());
}

TEST(FigureDescriptorTest, PointFactory) {
    auto desc = FigureDescriptor::point(10.0, 20.0);

    EXPECT_EQ(desc.type, FigureType::ET_POINT2D);
    EXPECT_TRUE(desc.x.has_value());
    EXPECT_TRUE(desc.y.has_value());
    EXPECT_DOUBLE_EQ(desc.x.value(), 10.0);
    EXPECT_DOUBLE_EQ(desc.y.value(), 20.0);
    EXPECT_TRUE(desc.pointIds.empty());
}

TEST(FigureDescriptorTest, LineFactory) {
    auto desc = FigureDescriptor::line(ID(1), ID(2));

    EXPECT_EQ(desc.type, FigureType::ET_LINE);
    EXPECT_EQ(desc.pointIds.size(), 2);
    EXPECT_EQ(desc.pointIds[0], ID(1));
    EXPECT_EQ(desc.pointIds[1], ID(2));
}

TEST(FigureDescriptorTest, CircleFactory) {
    auto desc = FigureDescriptor::circle(ID(5), 15.0);

    EXPECT_EQ(desc.type, FigureType::ET_CIRCLE);
    EXPECT_EQ(desc.pointIds.size(), 1);
    EXPECT_EQ(desc.pointIds[0], ID(5));
    EXPECT_TRUE(desc.radius.has_value());
    EXPECT_DOUBLE_EQ(desc.radius.value(), 15.0);
}

TEST(FigureDescriptorTest, ArcFactory) {
    auto desc = FigureDescriptor::arc(ID(1), ID(2), ID(3));

    EXPECT_EQ(desc.type, FigureType::ET_ARC);
    EXPECT_EQ(desc.pointIds.size(), 3);
    EXPECT_EQ(desc.pointIds[0], ID(1));
    EXPECT_EQ(desc.pointIds[1], ID(2));
    EXPECT_EQ(desc.pointIds[2], ID(3));
}

TEST(FigureDescriptorTest, ValidatePoint) {
    auto validPoint = FigureDescriptor::point(0.0, 0.0);
    EXPECT_TRUE(validPoint.validate());

    FigureDescriptor invalidPoint;
    invalidPoint.type = FigureType::ET_POINT2D;
    EXPECT_THROW(invalidPoint.validate(), std::invalid_argument);
}

TEST(FigureDescriptorTest, ValidateLine) {
    auto validLine = FigureDescriptor::line(ID(1), ID(2));
    EXPECT_TRUE(validLine.validate());

    FigureDescriptor invalidLine;
    invalidLine.type = FigureType::ET_LINE;
    invalidLine.pointIds = {ID(1)};
    EXPECT_THROW(invalidLine.validate(), std::invalid_argument);
}

TEST(FigureDescriptorTest, ValidateCircle) {
    auto validCircle = FigureDescriptor::circle(ID(1), 10.0);
    EXPECT_TRUE(validCircle.validate());

    FigureDescriptor noCenter;
    noCenter.type = FigureType::ET_CIRCLE;
    noCenter.radius = 10.0;
    EXPECT_THROW(noCenter.validate(), std::invalid_argument);

    FigureDescriptor noRadius;
    noRadius.type = FigureType::ET_CIRCLE;
    noRadius.pointIds = {ID(1)};
    EXPECT_THROW(noRadius.validate(), std::invalid_argument);

    FigureDescriptor zeroRadius;
    zeroRadius.type = FigureType::ET_CIRCLE;
    zeroRadius.pointIds = {ID(1)};
    zeroRadius.radius = 0.0;
    EXPECT_THROW(zeroRadius.validate(), std::invalid_argument);

    FigureDescriptor negativeRadius;
    negativeRadius.type = FigureType::ET_CIRCLE;
    negativeRadius.pointIds = {ID(1)};
    negativeRadius.radius = -5.0;
    EXPECT_THROW(negativeRadius.validate(), std::invalid_argument);
}

TEST(FigureDescriptorTest, ValidateArc) {
    auto validArc = FigureDescriptor::arc(ID(1), ID(2), ID(3));
    EXPECT_TRUE(validArc.validate());

    FigureDescriptor invalidArc;
    invalidArc.type = FigureType::ET_ARC;
    invalidArc.pointIds = {ID(1), ID(2)};
    EXPECT_THROW(invalidArc.validate(), std::invalid_argument);
}

TEST(PointUpdateDescriptorTest, Construction) {
    PointUpdateDescriptor desc(ID(5), 10.0, 20.0);

    EXPECT_EQ(desc.pointId, ID(5));
    EXPECT_TRUE(desc.newX.has_value());
    EXPECT_TRUE(desc.newY.has_value());
    EXPECT_DOUBLE_EQ(desc.newX.value(), 10.0);
    EXPECT_DOUBLE_EQ(desc.newY.value(), 20.0);
}

TEST(PointUpdateDescriptorTest, PartialUpdate) {
    PointUpdateDescriptor xOnly(ID(1), 5.0, std::nullopt);
    EXPECT_TRUE(xOnly.newX.has_value());
    EXPECT_FALSE(xOnly.newY.has_value());

    PointUpdateDescriptor yOnly(ID(2), std::nullopt, 10.0);
    EXPECT_FALSE(yOnly.newX.has_value());
    EXPECT_TRUE(yOnly.newY.has_value());
}

TEST(CircleUpdateDescriptorTest, Construction) {
    CircleUpdateDescriptor desc(ID(7), 25.0);

    EXPECT_EQ(desc.circleId, ID(7));
    EXPECT_DOUBLE_EQ(desc.newRadius, 25.0);
}
