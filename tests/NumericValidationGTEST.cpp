#include <gtest/gtest.h>
#include "DCMManager.h"
#include "RequirementFunction.h"
#include <array>
#include <limits>
#include <numbers>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
const std::array invalidNumbers = {std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity()};

void expectSameFigures(const DCMManager& manager, const std::vector<FigureDescriptor>& before) {
    ASSERT_EQ(manager.figureCount(), before.size());
    for (const auto& figure : before) {
        const auto after = manager.getFigure(*figure.id);
        ASSERT_TRUE(after);
        EXPECT_EQ(after->coords, figure.coords);
        EXPECT_EQ(after->x, figure.x);
        EXPECT_EQ(after->y, figure.y);
        EXPECT_EQ(after->radius, figure.radius);
        EXPECT_EQ(after->pointIds, figure.pointIds);
    }
}
}

TEST(NumericValidationTest, CreationRejectsEveryNonfiniteCoordinateBeforeAllocatingPoints) {
    DCMManager manager;
    for (double invalid : invalidNumbers) {
        for (auto descriptor : {FigureDescriptor::point(1, 2), FigureDescriptor::line(1, 2, 3, 4),
                                FigureDescriptor::circle(1, 2, 3), FigureDescriptor::arc(1, 2, 3, 4, 5, 6)}) {
            for (std::size_t coordinate = 0; coordinate < descriptor.coords.size(); ++coordinate) {
                auto malformed = descriptor;
                malformed.coords[coordinate] = invalid;
                EXPECT_THROW(manager.addFigure(malformed), std::invalid_argument);
                EXPECT_EQ(manager.figureCount(), 0u);
                EXPECT_EQ(manager.getComponentCount(), 0u);
            }
        }
        auto descriptor = FigureDescriptor::point(1, 2);
        descriptor.x = invalid; // Also validate redundant fields, even when coords takes precedence.
        EXPECT_THROW(manager.addFigure(descriptor), std::invalid_argument);
        descriptor.x = 1; descriptor.y = invalid;
        EXPECT_THROW(manager.addFigure(descriptor), std::invalid_argument);
        EXPECT_THROW(manager.addFigure(FigureDescriptor::circle(1, 2, invalid)), std::invalid_argument);
    }
    for (double radius : {-1.0, 0.0}) {
        EXPECT_THROW(manager.addFigure(FigureDescriptor::circle(1, 2, radius)), std::invalid_argument);
    }
    EXPECT_EQ(manager.addFigure(FigureDescriptor::point(1, 2)), ID(1));
}

TEST(NumericValidationTest, TypedAndUnifiedUpdatesRejectBadDataWithoutChangingGeometry) {
    DCMManager manager;
    const auto point = manager.addFigure(FigureDescriptor::point(1, 2));
    const auto line = manager.addFigure(FigureDescriptor::line(3, 4, 5, 6));
    const auto circle = manager.addFigure(FigureDescriptor::circle(7, 8, 2));
    const auto arc = manager.addFigure(FigureDescriptor::arc(0, 1, 1, 0, 0, 0));
    const auto before = manager.getAllFigures();
    for (double invalid : invalidNumbers) {
        for (const auto update : {PointUpdateDescriptor(point, invalid, 10),
                                  PointUpdateDescriptor(point, 10, invalid)}) {
            EXPECT_THROW(manager.updatePoint(update), std::invalid_argument);
        }
        for (int coordinate = 0; coordinate < 4; ++coordinate) {
            std::array<double, 4> coordinates{10, 11, 12, 13};
            coordinates[coordinate] = invalid;
            EXPECT_THROW(manager.updateLine({line, coordinates[0], coordinates[1], coordinates[2], coordinates[3]}),
                         std::invalid_argument);
        }
        for (int coordinate = 0; coordinate < 6; ++coordinate) {
            std::array<double, 6> coordinates{10, 11, 12, 13, 14, 15};
            coordinates[coordinate] = invalid;
            EXPECT_THROW(manager.updateArc({arc, coordinates[0], coordinates[1], coordinates[2],
                                            coordinates[3], coordinates[4], coordinates[5]}), std::invalid_argument);
        }
        EXPECT_THROW(manager.updateCircle(CircleUpdateDescriptor::center(circle, invalid, 10)), std::invalid_argument);
        EXPECT_THROW(manager.updateCircle(CircleUpdateDescriptor::center(circle, 10, invalid)), std::invalid_argument);
        EXPECT_THROW(manager.updateCircle({circle, invalid}), std::invalid_argument);
        EXPECT_THROW(manager.updateFigure(FigureUpdateDescriptor::circleRadius(circle, invalid)), std::invalid_argument);
        for (auto update : {FigureUpdateDescriptor::point(point, 10, 11),
                            FigureUpdateDescriptor::line(line, 10, 11, 12, 13),
                            FigureUpdateDescriptor::circle(circle, 10, 11, 3),
                            FigureUpdateDescriptor::arc(arc, 10, 11, 12, 13, 14, 15)}) {
            for (std::size_t coordinate = 0; coordinate < update.coords.size(); ++coordinate) {
                auto malformed = update;
                malformed.coords[coordinate] = invalid;
                EXPECT_THROW(manager.updateFigure(malformed), std::invalid_argument);
            }
            update.x = invalid;
            EXPECT_THROW(manager.updateFigure(update), std::invalid_argument);
        }
        expectSameFigures(manager, before);
    }
    for (double radius : {-1.0, 0.0}) {
        EXPECT_THROW(manager.updateCircle({circle, radius}), std::invalid_argument);
        EXPECT_THROW(manager.updateFigure(FigureUpdateDescriptor::circleRadius(circle, radius)), std::invalid_argument);
    }
    expectSameFigures(manager, before);
}

TEST(NumericValidationTest, InvalidBatchDoesNotApplyItsValidPrefix) {
    DCMManager manager;
    const auto point = manager.addFigure(FigureDescriptor::point(1, 2));
    const auto line = manager.addFigure(FigureDescriptor::line(3, 4, 5, 6));
    const auto circle = manager.addFigure(FigureDescriptor::circle(7, 8, 2));
    const auto arc = manager.addFigure(FigureDescriptor::arc(0, 1, 1, 0, 0, 0));
    const auto before = manager.getAllFigures();
    const double invalid = invalidNumbers[0];
    EXPECT_THROW(manager.updatePoints({{point, 10, 11}, {point, invalid, 1}}), std::invalid_argument);
    EXPECT_THROW(manager.updateLines({{line, 10, 11, 12, 13}, {line, 1, 2, 3, invalid}}), std::invalid_argument);
    EXPECT_THROW(manager.updateCircles({{circle, 3}, {circle, -1}}), std::invalid_argument);
    EXPECT_THROW(manager.updateArcs({{arc, 10, 11, 12, 13, 14, 15}, {arc, 1, 2, 3, 4, 5, invalid}}),
                 std::invalid_argument);
    EXPECT_THROW(manager.updateFigures({FigureUpdateDescriptor::point(point, 10, 11),
                                        FigureUpdateDescriptor::circleRadius(circle, invalid)}), std::invalid_argument);
    expectSameFigures(manager, before);
}

TEST(NumericValidationTest, ConstraintParametersAreCheckedOnAddAndUpdate) {
    DCMManager manager;
    const auto point = manager.addFigure(FigureDescriptor::point(0, 0));
    const auto second = manager.addFigure(FigureDescriptor::point(3, 4));
    const auto line = manager.addFigure(FigureDescriptor::line(point, second));
    const auto otherLine = manager.addFigure(FigureDescriptor::line(0, 1, 3, 5));
    const auto circle = manager.addFigure(FigureDescriptor::circle(0, 1, 2));
    const auto before = manager.getAllFigures();
    for (auto descriptor : {RequirementDescriptor::pointLineDist(point, line, 1),
                            RequirementDescriptor::pointPointDist(point, second, 5),
                            RequirementDescriptor::lineCircleDist(line, circle, 1),
                            RequirementDescriptor::lineLineAngle(line, otherLine, 1)}) {
        const auto requirement = manager.addRequirement(descriptor);
        const auto count = manager.requirementCount();
        for (double invalid : {invalidNumbers[0], invalidNumbers[1], invalidNumbers[2], -1.0}) {
            descriptor.param = invalid;
            EXPECT_THROW(manager.addRequirement(descriptor), std::invalid_argument);
            EXPECT_EQ(manager.requirementCount(), count);
            EXPECT_THROW(manager.updateRequirementParam(requirement, invalid), std::invalid_argument);
            EXPECT_DOUBLE_EQ(*manager.getRequirement(requirement)->param,
                             descriptor.type == RequirementType::ET_POINTPOINTDIST ? 5.0 : 1.0);
        }
    }
    EXPECT_THROW(manager.addRequirement(RequirementDescriptor::lineLineAngle(line, otherLine, std::numbers::pi + 0.01)),
                 std::invalid_argument);
    const auto angleId = manager.addRequirement(RequirementDescriptor::lineLineAngle(line, otherLine, 1.0));
    EXPECT_THROW(manager.updateRequirementParam(angleId, std::numbers::pi + 0.01), std::invalid_argument);
    EXPECT_DOUBLE_EQ(*manager.getRequirement(angleId)->param, 1.0);
    const auto distanceId = manager.addRequirement(RequirementDescriptor::pointPointDist(point, second, 5.0));
    for (double weight : {invalidNumbers[0], invalidNumbers[1], invalidNumbers[2], -1.0}) {
        EXPECT_THROW(manager.updateRequirementWeight(distanceId, weight), std::invalid_argument);
        EXPECT_DOUBLE_EQ(manager.getRequirement(distanceId)->weight, 1.0);
    }
    auto unexpected = RequirementDescriptor::horizontal(line);
    unexpected.param = 1;
    EXPECT_THROW(manager.addRequirement(unexpected), std::invalid_argument);
    for (double angle : {0.0, std::numbers::pi}) {
        EXPECT_TRUE(RequirementDescriptor::lineLineAngle(line, otherLine, angle).validate());
    }
    for (double distance : {0.0, 1.0}) {
        EXPECT_TRUE(RequirementDescriptor::pointPointDist(point, second, distance).validate());
    }
    expectSameFigures(manager, before);
}

TEST(NumericValidationTest, StorageRejectsInvalidNumbersWithoutConsumingIdsOrCreatingNestedPoints) {
    Figures::GeometryStorage storage;
    for (double invalid : invalidNumbers) {
        EXPECT_THROW((void)storage.createPoint(1, invalid), std::invalid_argument);
        EXPECT_EQ(storage.currentID(), ID(1));
    }
    const auto center = storage.createPoint(0, 0);
    const auto nextId = storage.currentID();
    for (double radius : {invalidNumbers[0], invalidNumbers[1], invalidNumbers[2], -1.0, 0.0}) {
        EXPECT_THROW((void)storage.createCircle(center, radius), std::invalid_argument);
        Figures::FigureData data;
        data.radius = radius;
        EXPECT_THROW((void)storage.createFigure(FigureType::ET_CIRCLE, data), std::invalid_argument);
        EXPECT_EQ(storage.currentID(), nextId);
        EXPECT_EQ(storage.size(), 1u);
    }
    for (const auto type : {FigureType::ET_LINE, FigureType::ET_ARC}) {
        Figures::FigureData data;
        data.points = {{1, 2}, {3, invalidNumbers[0]}};
        EXPECT_THROW((void)storage.createFigure(type, data), std::invalid_argument);
        EXPECT_EQ(storage.currentID(), nextId);
        EXPECT_EQ(storage.size(), 1u);
    }
}

TEST(NumericValidationTest, DirectFunctionsAndFiguresRejectInvalidNumericConstruction) {
    double x = 0, y = 0, x2 = 1, y2 = 1, cx = 2, cy = 2, radius = 1;
    for (double invalid : {invalidNumbers[0], invalidNumbers[1], invalidNumbers[2], -1.0}) {
        EXPECT_THROW(Function::PointPointDistanceFunction({&x, &y, &x2, &y2}, invalid), std::invalid_argument);
        EXPECT_THROW(Function::PointLineDistanceFunction({&cx, &cy, &x, &y, &x2, &y2}, invalid), std::invalid_argument);
        EXPECT_THROW(Function::LineCircleDistanceFunction({&x, &y, &x2, &y2, &cx, &cy, &radius}, invalid),
                     std::invalid_argument);
        EXPECT_THROW(Function::LineLineAngleFunction({&x, &y, &x2, &y2, &x, &y, &cx, &cy}, invalid),
                     std::invalid_argument);
    }
    Figures::Point2D center(0, 0);
    for (double invalid : invalidNumbers) {
        EXPECT_THROW(Figures::Point2D(invalid, 1), std::invalid_argument);
        EXPECT_THROW(Figures::Circle2D(&center, invalid), std::invalid_argument);
        EXPECT_THROW(Function::FixCoordinateFunction(RequirementType::ET_FIXPOINT, {&x}, invalid),
                     std::invalid_argument);
    }
    EXPECT_THROW(Figures::Circle2D(&center, -1), std::invalid_argument);
    EXPECT_THROW(Figures::Circle2D(&center, 0), std::invalid_argument);
    x = invalidNumbers[0];
    EXPECT_THROW(Function::PointPointDistanceFunction({&x, &y, &x2, &y2}, 1), std::invalid_argument);
    center.x() = invalidNumbers[0];
    EXPECT_THROW(Figures::Line2D(&center, &center), std::invalid_argument);
    EXPECT_THROW(Figures::Circle2D(&center, 1), std::invalid_argument);
    EXPECT_THROW(Figures::Arc2D(&center, &center, &center), std::invalid_argument);
}

TEST(NumericValidationTest, DirectRequirementSystemRejectsBadParametersWithoutChangingExistingSystem) {
    Figures::GeometryStorage storage;
    const auto first = storage.createPoint(0, 0);
    const auto second = storage.createPoint(3, 4);
    System::RequirementSystem system(&storage);
    const auto id = system.addRequirement(RequirementDescriptor::pointPointDist(first, second, 5));
    auto invalid = RequirementDescriptor::pointPointDist(first, second, 5);
    invalid.param = invalidNumbers[0];
    EXPECT_THROW(system.addRequirement(invalid), std::invalid_argument);
    EXPECT_EQ(system.getRequirements().size(), 1u);
    ASSERT_TRUE(system.getRequirement(id));
    EXPECT_EQ(system.getRequirementParam(id), 5.0);
    EXPECT_NEAR(system.residuals().norm(), 0.0, 1e-12);
    EXPECT_EQ(system.addRequirement(RequirementDescriptor::pointPointDist(first, second, 5)), ID(id.id + 1));
}

TEST(NumericValidationTest, RestoringInvalidSnapshotKeepsCurrentDocument) {
    DCMManager source;
    const auto point = source.addFigure(FigureDescriptor::point(1, 2));
    source.storage().get<Figures::Point2D>(point)->x() = invalidNumbers[0];
    const auto invalidState = source.snapshot();
    DCMManager destination;
    destination.addFigure(FigureDescriptor::circle(3, 4, 5));
    const auto before = destination.getAllFigures();
    EXPECT_THROW(destination.restoreSnapshot(invalidState), std::invalid_argument);
    expectSameFigures(destination, before);
}
