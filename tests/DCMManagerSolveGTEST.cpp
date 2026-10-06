#include <gtest/gtest.h>
#include "DCMManager.h"
#include "../math/tests/support/FaultInjectionConstraint.h"
#include <algorithm>
#include <cmath>
#include <array>
#include <limits>
#include <optional>
#include <utility>

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

class DCMManagerSolveTest : public ::testing::Test {
protected:
    DCMManager manager;
};

TEST(DCMManagerDegenerateSolveTest, PointOnLineAndDistanceUseCollapsedPointInEveryMode) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (const bool onLine : {false, true}) {
            for (const auto position : {std::array<double, 2>{100, 100}, {0, 0}, {3, 4}}) {
                SCOPED_TRACE(static_cast<int>(mode));
                SCOPED_TRACE(onLine);
                SCOPED_TRACE(position[0]);
                DCMManager manager;
                const auto line = manager.addFigure(FigureDescriptor::line(0, 0, 0, 0));
                const auto point = manager.addFigure(FigureDescriptor::point(position[0], position[1]));
                manager.addRequirement(RequirementDescriptor::fixLine(line));
                manager.addRequirement(RequirementDescriptor::fixPoint(point));
                manager.addRequirement(onLine ? RequirementDescriptor::pointOnLine(point, line)
                                             : RequirementDescriptor::pointLineDist(point, line, 5));
                manager.setSolveMode(mode);
                const auto component = manager.getComponentForFigure(line);
                ASSERT_TRUE(component);
                const bool expected = onLine ? position[0] == 0 : position[0] == 3;
                // Repeat to also exercise the cached pipeline.
                EXPECT_EQ(manager.solve(*component), expected);
                EXPECT_EQ(manager.solve(*component), expected);
                const auto unchanged = manager.getFigure(point);
                ASSERT_TRUE(unchanged);
                EXPECT_DOUBLE_EQ(*unchanged->x, position[0]);
                EXPECT_DOUBLE_EQ(*unchanged->y, position[1]);
            }
        }
    }
}

TEST(DCMManagerDegenerateSolveTest, FreePointCanMoveOntoCollapsedFixedLine) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager manager;
        const auto line = manager.addFigure(FigureDescriptor::line(0, 0, 0, 0));
        const auto point = manager.addFigure(FigureDescriptor::point(100, 100));
        manager.addRequirement(RequirementDescriptor::fixLine(line));
        manager.addRequirement(RequirementDescriptor::pointOnLine(point, line));
        manager.setSolveMode(mode);
        const auto component = manager.getComponentForFigure(line);
        ASSERT_TRUE(component);
        ASSERT_TRUE(manager.solve(*component));
        const auto result = manager.getFigure(point);
        ASSERT_TRUE(result);
        EXPECT_LE(std::hypot(*result->x, *result->y), 1e-4);
        if (mode == SolveMode::DRAG) {
            // Automatic solve uses temporary locks and then retries without locks.
            manager.updatePoint({point, 100, 100});
            const auto dragged = manager.getFigure(point);
            ASSERT_TRUE(dragged);
            EXPECT_LE(std::hypot(*dragged->x, *dragged->y), 1e-4);
            EXPECT_TRUE(manager.solve(*component));
        }
    }
}

TEST(DCMManagerDegenerateSolveTest, UndefinedLineDirectionsFailWithFixedOrFreeVariables) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (const auto type : {RequirementType::ET_LINELINEPARALLEL,
                               RequirementType::ET_LINELINEPERPENDICULAR,
                               RequirementType::ET_LINELINEANGLE,
                               RequirementType::ET_HORIZONTAL, RequirementType::ET_VERTICAL}) {
            for (int collapsed : {0, 1, 2}) {
                if (collapsed == 1 && (type == RequirementType::ET_HORIZONTAL ||
                                       type == RequirementType::ET_VERTICAL)) continue;
                for (const bool fixed : {false, true}) {
                    SCOPED_TRACE(static_cast<int>(mode));
                    SCOPED_TRACE(static_cast<int>(type));
                    SCOPED_TRACE(collapsed);
                    SCOPED_TRACE(fixed);
                    DCMManager manager;
                    const auto first = manager.addFigure(FigureDescriptor::line(0, 0, collapsed == 1 ? 2 : 0, 0));
                    const auto second = manager.addFigure(FigureDescriptor::line(0, 0, 0, collapsed == 0 ? 2 : 0));
                    if (fixed) {
                        manager.addRequirement(RequirementDescriptor::fixLine(first));
                        manager.addRequirement(RequirementDescriptor::fixLine(second));
                    }
                    const bool unary = type == RequirementType::ET_HORIZONTAL || type == RequirementType::ET_VERTICAL;
                    const auto requirement = manager.addRequirement(RequirementDescriptor(
                        type, unary ? std::vector<ID>{first} : std::vector<ID>{first, second},
                        type == RequirementType::ET_LINELINEANGLE ? std::optional<double>(1) : std::nullopt));
                    manager.setSolveMode(mode);
                    const auto component = manager.getComponentForFigure(first);
                    ASSERT_TRUE(component);
                    for (int attempt = 0; attempt < 2; ++attempt) {
                        EXPECT_FALSE(manager.solve(*component));
                        EXPECT_TRUE(manager.getRequirementSystem().J().toDense().allFinite());
                    }
                    // A disabled undefined requirement must not inject 0 * infinity into LM.
                    manager.updateRequirementWeight(requirement, 0);
                    EXPECT_TRUE(manager.solve(*component));
                    EXPECT_TRUE(manager.getRequirementSystem().residuals().allFinite());
                }
            }
        }
    }
}

TEST(DCMManagerDegenerateSolveTest, ShortFixedLinesDoNotFalselySatisfyDirectionConstraints) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (const auto type : {RequirementType::ET_LINELINEPARALLEL,
                               RequirementType::ET_LINELINEPERPENDICULAR, RequirementType::ET_LINELINEANGLE}) {
            DCMManager manager;
            const auto first = manager.addFigure(FigureDescriptor::line(0, 0, 5e-11, 0));
            const auto second = manager.addFigure(type == RequirementType::ET_LINELINEPARALLEL
                ? FigureDescriptor::line(0, 0, 0, 5e-11) : FigureDescriptor::line(0, 0, 5e-11, 0));
            manager.addRequirement(RequirementDescriptor::fixLine(first));
            manager.addRequirement(RequirementDescriptor::fixLine(second));
            manager.addRequirement(RequirementDescriptor(type, {first, second},
                type == RequirementType::ET_LINELINEANGLE ? std::optional<double>(1) : std::nullopt));
            manager.setSolveMode(mode);
            const auto component = manager.getComponentForFigure(first);
            ASSERT_TRUE(component);
            EXPECT_FALSE(manager.solve(*component));
        }
    }
}

TEST(DCMManagerDegenerateSolveTest, AliasingCanCollapseLineAndGeometryUpdatesCanRestoreDirection) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager manager;
        const auto a = manager.addFigure(FigureDescriptor::point(0, 0));
        const auto b = manager.addFigure(FigureDescriptor::point(2, 0));
        const auto line = manager.addFigure(FigureDescriptor::line(a, b));
        manager.addRequirement(RequirementDescriptor::horizontal(line));
        const auto coincidence = manager.addRequirement(RequirementDescriptor::pointOnPoint(a, b));
        manager.setSolveMode(mode);
        const auto component = manager.getComponentForFigure(line);
        ASSERT_TRUE(component);
        EXPECT_FALSE(manager.solve(*component));
        manager.removeRequirement(coincidence);
        manager.updatePoint({a, 0, 0}); // Aliasing may have copied the representative into a.
        manager.updatePoint({b, 0, 0}); // Includes an automatic DRAG solve at zero length.
        const auto refreshed = manager.getComponentForFigure(line);
        ASSERT_TRUE(refreshed);
        EXPECT_FALSE(manager.solve(*refreshed));
        manager.updatePoint({b, 2, 0});
        EXPECT_TRUE(manager.solve(*refreshed));
        manager.updatePoint({b, 0, 0});
        EXPECT_FALSE(manager.solve(*refreshed)); // Cached pipeline must reevaluate validity.
    }
}

TEST(DCMManagerDegenerateSolveTest, ArcRejectsCollapsedChordAndOffBisectorCenter) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (const double length : {0.0, 5e-13, 5e-11}) {
            DCMManager manager;
            const auto arc = manager.addFigure(FigureDescriptor::arc(0, 0, length, 0, 100, 2));
            const auto descriptor = manager.getFigure(arc);
            ASSERT_TRUE(descriptor);
            for (const auto point : descriptor->pointIds) {
                manager.addRequirement(RequirementDescriptor::fixPoint(point));
            }
            manager.addRequirement(RequirementDescriptor::arcCenterOnPerpendicular(arc));
            manager.setSolveMode(mode);
            const auto component = manager.getComponentForFigure(arc);
            ASSERT_TRUE(component);
            EXPECT_FALSE(manager.solve(*component));
        }
    }
}

TEST(DCMManagerDegenerateSolveTest, FixedAssignmentsCanCollapseLineAndRejectedSolveRestoresInput) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        DCMManager manager;
        const auto line = manager.addFigure(FigureDescriptor::line(0, 0, 0, 0));
        manager.addRequirement(RequirementDescriptor::fixLine(line));
        manager.addRequirement(RequirementDescriptor::horizontal(line));
        const auto descriptor = manager.getFigure(line);
        ASSERT_TRUE(descriptor);
        const auto endpoint = descriptor->pointIds[1];
        // Public updates respect fixation; the mutable storage API can bypass it.
        *manager.storage().get<Figures::Point2D>(endpoint)->ptrX() = 2;
        manager.setSolveMode(mode);
        const auto component = manager.getComponentForFigure(line);
        ASSERT_TRUE(component);
        for (int attempt = 0; attempt < 2; ++attempt) {
            EXPECT_FALSE(manager.solve(*component));
            const auto restored = manager.getFigure(endpoint);
            ASSERT_TRUE(restored);
            EXPECT_DOUBLE_EQ(*restored->x, 2);
            EXPECT_DOUBLE_EQ(*restored->y, 0);
        }
    }
}

TEST(DCMManagerDegenerateSolveTest, DefinedPointAndCircleConstraintsUseActualResidualInEveryMode) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        for (const auto type : {RequirementType::ET_POINTPOINTDIST, RequirementType::ET_LINECIRCLEDIST,
                               RequirementType::ET_LINEONCIRCLE}) {
            for (const bool satisfied : {false, true}) {
                SCOPED_TRACE(static_cast<int>(mode));
                SCOPED_TRACE(static_cast<int>(type));
                SCOPED_TRACE(satisfied);
                DCMManager manager;
                ID subject;
                if (type == RequirementType::ET_POINTPOINTDIST) {
                    const auto first = manager.addFigure(FigureDescriptor::point(0, 0));
                    const auto second = manager.addFigure(FigureDescriptor::point(0, 0));
                    subject = first;
                    manager.addRequirement(RequirementDescriptor::fixPoint(first));
                    manager.addRequirement(RequirementDescriptor::fixPoint(second));
                    manager.addRequirement(RequirementDescriptor::pointPointDist(first, second, satisfied ? 0 : 1));
                } else {
                    const auto line = manager.addFigure(FigureDescriptor::line(3, 4, 3, 4));
                    const auto circle = manager.addFigure(FigureDescriptor::circle(0, 0,
                        type == RequirementType::ET_LINEONCIRCLE ? (satisfied ? 5 : 4) : 2));
                    subject = line;
                    manager.addRequirement(RequirementDescriptor::fixLine(line));
                    manager.addRequirement(RequirementDescriptor::fixCircle(circle));
                    manager.addRequirement(type == RequirementType::ET_LINEONCIRCLE
                        ? RequirementDescriptor::lineOnCircle(line, circle)
                        : RequirementDescriptor::lineCircleDist(line, circle, satisfied ? 3 : 4));
                }
                manager.setSolveMode(mode);
                const auto component = manager.getComponentForFigure(subject);
                ASSERT_TRUE(component);
                EXPECT_EQ(manager.solve(*component), satisfied);
            }
        }
        DCMManager manager;
        const auto collapsed = manager.addFigure(FigureDescriptor::line(0, 0, 0, 0));
        manager.addRequirement(RequirementDescriptor::fixLine(collapsed));
        manager.setSolveMode(mode);
        const auto component = manager.getComponentForFigure(collapsed);
        ASSERT_TRUE(component);
        EXPECT_TRUE(manager.solve(*component)); // Fixation alone needs no direction.
    }
}

TEST(DCMManagerDegenerateSolveTest, LocalAndDragOnlyRejectUndefinedGeometryInSelectedComponent) {
    DCMManager manager;
    const auto invalid = manager.addFigure(FigureDescriptor::line(0, 0, 0, 0));
    const auto valid = manager.addFigure(FigureDescriptor::line(10, 0, 20, 0));
    manager.addRequirement(RequirementDescriptor::horizontal(invalid));
    manager.addRequirement(RequirementDescriptor::horizontal(valid));
    const auto invalidComponent = manager.getComponentForFigure(invalid);
    const auto validComponent = manager.getComponentForFigure(valid);
    ASSERT_TRUE(invalidComponent && validComponent);
    ASSERT_NE(*invalidComponent, *validComponent);
    for (const auto mode : {SolveMode::LOCAL, SolveMode::DRAG}) {
        manager.setSolveMode(mode);
        EXPECT_TRUE(manager.solve(*validComponent));
        EXPECT_FALSE(manager.solve(*invalidComponent));
    }
    manager.setSolveMode(SolveMode::GLOBAL);
    EXPECT_FALSE(manager.solve());
}

TEST(DCMManagerSolveInvariantTest, NegativeRadiusIsRejectedAndOriginalGeometryIsRestored) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        SCOPED_TRACE(static_cast<int>(mode));
        DCMManager manager;
        const auto line = manager.addFigure(FigureDescriptor::line(-2, 0, 2, 0));
        const auto center = manager.addFigure(FigureDescriptor::point(0, 0));
        const auto circle = manager.addFigure(FigureDescriptor::circle(center, 2));
        const auto moving = manager.addFigure(FigureDescriptor::point(3, 0));
        const auto alias = manager.addFigure(FigureDescriptor::point(3, 0));
        manager.addRequirement(RequirementDescriptor::fixLine(line));
        const auto fixedCenter = manager.addRequirement(RequirementDescriptor::fixPoint(center));
        manager.addRequirement(RequirementDescriptor::lineCircleDist(line, circle, 1));
        manager.addRequirement(RequirementDescriptor::pointPointDist(center, moving, 5));
        manager.addRequirement(RequirementDescriptor::pointOnPoint(moving, alias));
        const auto* originalCircle = manager.getStorage().get<Figures::Circle2D>(circle);
        const auto figureCount = manager.figureCount();
        manager.setSolveMode(mode);
        const auto component = manager.getComponentForFigure(circle);
        ASSERT_TRUE(component);

        for (int attempt = 0; attempt < 2; ++attempt) {
            EXPECT_FALSE(manager.solve(*component));
            const auto restored = manager.getFigure(circle);
            ASSERT_TRUE(restored && restored->radius);
            EXPECT_DOUBLE_EQ(*restored->radius, 2.0);
            EXPECT_EQ(manager.getStorage().get<Figures::Circle2D>(circle), originalCircle);
            EXPECT_EQ(manager.figureCount(), figureCount);
            const auto restoredCenter = manager.getFigure(center);
            ASSERT_TRUE(restoredCenter);
            EXPECT_DOUBLE_EQ(*restoredCenter->x, 0.0);
            EXPECT_DOUBLE_EQ(*restoredCenter->y, 0.0);
            for (const auto id : {moving, alias}) {
                const auto restoredPoint = manager.getFigure(id);
                ASSERT_TRUE(restoredPoint);
                EXPECT_DOUBLE_EQ(*restoredPoint->x, 3.0);
                EXPECT_DOUBLE_EQ(*restoredPoint->y, 0.0);
            }
            EXPECT_NO_THROW(manager.getRequirementSystem().J());
            EXPECT_NO_THROW(manager.getRequirementSystem().diagnose());
        }

        manager.removeRequirement(fixedCenter);
        manager.setSolveMode(SolveMode::GLOBAL);
        manager.updatePoint({center, 0, 4});
        manager.setSolveMode(mode);
        const auto newComponent = manager.getComponentForFigure(circle);
        ASSERT_TRUE(newComponent);
        ASSERT_TRUE(manager.solve(*newComponent));
        const auto validCircle = manager.getFigure(circle);
        ASSERT_TRUE(validCircle && validCircle->radius);
        EXPECT_GT(*validCircle->radius, 0.0);
        EXPECT_TRUE(std::isfinite(*validCircle->radius));
    }
}

TEST(DCMManagerSolveInvariantTest, NonfiniteCandidatesAndExceptionsRestoreAliasedPoints) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        for (const bool throwOnCandidate : {false, true}) {
            SCOPED_TRACE(invalid);
            SCOPED_TRACE(throwOnCandidate);
            DCMManager manager;
            const auto first = manager.addFigure(FigureDescriptor::point(1, 2));
            const auto alias = manager.addFigure(FigureDescriptor::point(1, 2));
            manager.addRequirement(RequirementDescriptor::pointOnPoint(first, alias));
            auto* coordinate = manager.storage().get<Figures::Point2D>(first)->ptrX();
            auto& system = const_cast<System::RequirementSystem&>(manager.getRequirementSystem());
            system.addFunction(std::make_shared<OurPaintDCM::Function::RequirementFunction>(RequirementType::ET_POINTPOINTDIST,
                std::make_shared<FaultInjectionConstraint>(coordinate,invalid,throwOnCandidate)));
            if (throwOnCandidate) {
                EXPECT_THROW(manager.solve(), std::runtime_error);
            } else {
                EXPECT_FALSE(manager.solve());
            }
            for (const auto pointId : {first, alias}) {
                const auto point = manager.getFigure(pointId);
                ASSERT_TRUE(point);
                EXPECT_DOUBLE_EQ(*point->x, 1.0);
                EXPECT_DOUBLE_EQ(*point->y, 2.0);
            }
        }
    }
}

TEST(DCMManagerSolveInvariantTest, ZeroAndNonfiniteRadiusCandidatesAreRestored) {
    for (const double invalid : {0.0, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()}) {
        DCMManager manager;
        const auto line = manager.addFigure(FigureDescriptor::line(-2, 0, 2, 0));
        const auto circle = manager.addFigure(FigureDescriptor::circle(0, 4, 2));
        auto disabled = RequirementDescriptor::lineCircleDist(line, circle, 2);
        disabled.weight = 0;
        manager.addRequirement(disabled);
        auto* radius = manager.storage().get<Figures::Circle2D>(circle)->ptrRadius();
        auto& system = const_cast<System::RequirementSystem&>(manager.getRequirementSystem());
        system.addFunction(std::make_shared<OurPaintDCM::Function::RequirementFunction>(RequirementType::ET_POINTPOINTDIST,
            std::make_shared<FaultInjectionConstraint>(radius,invalid,false)));
        EXPECT_FALSE(manager.solve());
        EXPECT_DOUBLE_EQ(*radius, 2.0);
    }
}

TEST(DCMManagerSolveInvariantTest, EmptySystemChecksGeometryBeforeReportingSuccess) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        SCOPED_TRACE(static_cast<int>(mode));
        DCMManager manager;
        manager.setSolveMode(mode);
        EXPECT_TRUE(manager.solve()); // No figures and no constraints remains a valid empty system.
        const auto circleId = manager.addFigure(FigureDescriptor::circle(1, 2, 3));
        auto* circle = manager.storage().get<Figures::Circle2D>(circleId);
        ASSERT_NE(circle, nullptr);
        ASSERT_EQ(manager.requirementCount(), 0u);
        const auto component = manager.getComponentForFigure(circleId);
        ASSERT_TRUE(component);
        EXPECT_TRUE(manager.solve());
        for (const double invalid : {-1.0, 0.0, std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity()}) {
            // Simulate invalid stored data through the documented mutable escape hatch.
            circle->radius = invalid;
            EXPECT_FALSE(manager.solve());
            EXPECT_FALSE(manager.solve(*component));
        }
        circle->radius = 3;
        for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity()}) {
            for (double* coordinate : {circle->center->ptrX(), circle->center->ptrY()}) {
                const double original = *coordinate;
                *coordinate = invalid;
                EXPECT_FALSE(manager.solve());
                EXPECT_FALSE(manager.solve(*component));
                *coordinate = original;
            }
        }
        EXPECT_TRUE(manager.solve());
        EXPECT_TRUE(manager.solve(*component));
    }
}

TEST(DCMManagerSolveInvariantTest, AutomaticDragSolveRestoresRadiusAfterFallbackWithoutLocks) {
    DCMManager manager;
    const auto line = manager.addFigure(FigureDescriptor::line(-2, 0, 2, 0));
    const auto center = manager.addFigure(FigureDescriptor::point(0, 4));
    const auto circle = manager.addFigure(FigureDescriptor::circle(center, 2));
    manager.addRequirement(RequirementDescriptor::fixLine(line));
    manager.addRequirement(RequirementDescriptor::fixPoint(center));
    manager.addRequirement(RequirementDescriptor::lineCircleDist(line, circle, 5));
    manager.setSolveMode(SolveMode::DRAG);
    // Radius is the only free variable. Locking it forces the recursive fallback;
    // its unconstrained optimum would be -1. Preserve the valid user update.
    EXPECT_NO_THROW(manager.updateCircle({circle, 3}));
    const auto restored = manager.getFigure(circle);
    ASSERT_TRUE(restored && restored->radius);
    EXPECT_DOUBLE_EQ(*restored->radius, 3.0);
    const auto component = manager.getComponentForFigure(circle);
    ASSERT_TRUE(component);
    EXPECT_FALSE(manager.solve(*component));
    EXPECT_DOUBLE_EQ(*manager.getFigure(circle)->radius, 3.0);
}

TEST(DCMManagerLineCircleSolveTest, SolvesRequestedGapWithClosestPointInEachRegion) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL}) {
        for (const double centerX : {-5.0, 3.0, 12.0}) {
            SCOPED_TRACE(static_cast<int>(mode));
            SCOPED_TRACE(centerX);
            DCMManager manager;
            const auto line = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 6.0, 0.0));
            const auto center = manager.addFigure(FigureDescriptor::point(centerX, 4.0));
            const auto circle = manager.addFigure(FigureDescriptor::circle(center, 1.0));
            manager.addRequirement(RequirementDescriptor::fixLine(line));
            manager.addRequirement(RequirementDescriptor::fixPoint(center));
            manager.addRequirement(RequirementDescriptor::lineCircleDist(line, circle, 2.0));
            manager.setSolveMode(mode);
            const auto component = manager.getComponentForFigure(line);
            ASSERT_TRUE(component);
            const bool success = manager.solve(*component);
            const auto solved = manager.getFigure(circle);
            ASSERT_TRUE(solved && solved->radius);
            EXPECT_TRUE(success) << "radius=" << *solved->radius;
            const double nearestX = std::clamp(centerX, 0.0, 6.0);
            EXPECT_NEAR(std::hypot(centerX - nearestX, 4.0) - *solved->radius, 2.0, 1e-6);
            EXPECT_TRUE(manager.solve(*component));
            EXPECT_TRUE(manager.solve(*component, 1e-10));
            const auto refined = manager.getFigure(circle);
            ASSERT_TRUE(refined && refined->radius);
            EXPECT_NEAR(std::hypot(centerX - nearestX, 4.0) - *refined->radius, 2.0, 1e-10);
        }
    }
}

TEST(DCMManagerLineCircleSolveTest, BothEndpointsMustIndependentlyLieOnCircle) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL}) {
        for (const bool fixLine : {false, true}) {
            SCOPED_TRACE(static_cast<int>(mode));
            SCOPED_TRACE(fixLine);
            DCMManager manager;
            const auto center = manager.addFigure(FigureDescriptor::point(3.0, 7.0));
            const auto circle = manager.addFigure(FigureDescriptor::circle(center, 5.0));
            const auto line = manager.addFigure(FigureDescriptor::line(6.0, 7.0, 10.0, 7.0));
            manager.addRequirement(RequirementDescriptor::fixCircle(circle));
            if (fixLine) manager.addRequirement(RequirementDescriptor::fixLine(line));
            auto requirement = RequirementDescriptor::lineOnCircle(line, circle);
            requirement.weight = 2.0;
            manager.addRequirement(requirement);
            manager.setSolveMode(mode);
            const auto component = manager.getComponentForFigure(line);
            ASSERT_TRUE(component);
            EXPECT_EQ(manager.solve(*component), !fixLine);
            if (!fixLine) {
                const auto solvedLine = manager.getFigure(line);
                ASSERT_TRUE(solvedLine);
                for (const auto id : solvedLine->pointIds) {
                    const auto point = manager.getFigure(id);
                    ASSERT_TRUE(point);
                    EXPECT_NEAR(std::hypot(*point->x - 3.0, *point->y - 7.0), 5.0, 1e-6);
                }
                EXPECT_TRUE(manager.solve(*component));
            }
        }
    }
}

TEST(DCMManagerLineCircleSolveTest, MovesSegmentToRequestedGapFromFixedCircle) {
    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL}) {
        for (const double centerX : {-5.0, 3.0, 12.0}) {
            SCOPED_TRACE(static_cast<int>(mode));
            SCOPED_TRACE(centerX);
            DCMManager manager;
            const auto line = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 6.0, 0.0));
            const auto circle = manager.addFigure(FigureDescriptor::circle(centerX, 4.0, 1.0));
            manager.addRequirement(RequirementDescriptor::fixCircle(circle));
            manager.addRequirement(RequirementDescriptor::horizontal(line));
            manager.addRequirement(RequirementDescriptor::lineCircleDist(line, circle, 2.0));
            manager.setSolveMode(mode);
            const auto component = manager.getComponentForFigure(line);
            ASSERT_TRUE(component);
            ASSERT_TRUE(manager.solve(*component));
            const auto solved = manager.getFigure(line);
            ASSERT_TRUE(solved);
            const auto first = manager.getFigure(solved->pointIds[0]);
            const auto second = manager.getFigure(solved->pointIds[1]);
            ASSERT_TRUE(first && second);
            const double dx = *second->x - *first->x;
            const double dy = *second->y - *first->y;
            const double lengthSquared = dx * dx + dy * dy;
            ASSERT_GT(lengthSquared, 1e-12);
            const double t = std::clamp(((centerX - *first->x) * dx + (4.0 - *first->y) * dy)
                                           / lengthSquared, 0.0, 1.0);
            EXPECT_NEAR(std::hypot(*first->x + t * dx - centerX, *first->y + t * dy - 4.0),
                        3.0, 1e-6);
            EXPECT_NEAR(dy, 0.0, 1e-6);
        }
    }
}

TEST_F(DCMManagerSolveTest, DragMode_RepeatedMovesAndAbruptJumpStayStable) {
    const auto dragged = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto following = manager.addFigure(FigureDescriptor::point(3.0, 4.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(dragged, following, 5.0));
    manager.setSolveMode(SolveMode::DRAG);

    for (int event = 0; event < 150; ++event) {
        SCOPED_TRACE(event);
        const double offset = event < 120 ? 0.0 : 1000.0;
        const double x = offset + 0.1 * event;
        const double y = -offset + std::sin(0.1 * event);
        ASSERT_NO_THROW(manager.updatePoint(PointUpdateDescriptor(dragged, x, y)));
        const auto a = manager.getFigure(dragged);
        const auto b = manager.getFigure(following);
        ASSERT_TRUE(a && b);
        EXPECT_NEAR(a->x.value(), x, 1e-9);
        EXPECT_NEAR(a->y.value(), y, 1e-9);
        EXPECT_TRUE(std::isfinite(b->x.value()));
        EXPECT_TRUE(std::isfinite(b->y.value()));
        EXPECT_NEAR(std::hypot(b->x.value() - x, b->y.value() - y), 5.0, 1e-6);
    }
}

TEST_F(DCMManagerSolveTest, NonzeroResidualAtStationaryPointDoesNotReportSolved) {
    const auto line = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 0.0, 0.0));
    // A zero-length line has a zero Jacobian here, but violates the distance.
    const auto descriptor = manager.getFigure(line);
    ASSERT_TRUE(descriptor);
    manager.addRequirement(RequirementDescriptor::pointPointDist(
        descriptor->pointIds[0], descriptor->pointIds[1], 5.0));
    EXPECT_FALSE(manager.solve());
}

TEST(DCMManagerSharedPointSolveTest, LineConstraintsSolveWithOneSharedEndpoint) {
    enum class Constraint { Parallel, Perpendicular, Angle };
    const double targetAngle = std::acos(-1.0) / 3.0;

    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL}) {
        for (const auto constraint : {Constraint::Parallel, Constraint::Perpendicular,
                                      Constraint::Angle}) {
            SCOPED_TRACE(static_cast<int>(mode));
            SCOPED_TRACE(static_cast<int>(constraint));
            DCMManager manager;
            const auto a = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
            const auto shared = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
            const auto c = manager.addFigure(FigureDescriptor::point(13.0, 4.0));
            const auto first = manager.addFigure(FigureDescriptor::line(a, shared));
            const auto second = manager.addFigure(FigureDescriptor::line(shared, c));
            manager.addRequirement(RequirementDescriptor::fixLine(first));
            manager.addRequirement(RequirementDescriptor::pointPointDist(shared, c, 5.0));
            switch (constraint) {
                case Constraint::Parallel:
                    manager.addRequirement(RequirementDescriptor::lineLineParallel(first, second));
                    break;
                case Constraint::Perpendicular:
                    manager.addRequirement(RequirementDescriptor::lineLinePerpendicular(first, second));
                    break;
                case Constraint::Angle:
                    manager.addRequirement(RequirementDescriptor::lineLineAngle(first, second, targetAngle));
                    break;
            }

            manager.setSolveMode(mode);
            const auto component = manager.getComponentForFigure(first);
            ASSERT_TRUE(component.has_value());
            EXPECT_EQ(component, manager.getComponentForFigure(second));
            ASSERT_TRUE(manager.solve(*component));

            const auto line1 = manager.getFigure(first);
            const auto line2 = manager.getFigure(second);
            const auto p = manager.getFigure(shared);
            const auto end = manager.getFigure(c);
            ASSERT_TRUE(line1 && line2 && p && end);
            EXPECT_EQ(line1->pointIds[1], shared);
            EXPECT_EQ(line2->pointIds[0], shared);
            EXPECT_NEAR(p->x.value(), 10.0, 1e-8);
            EXPECT_NEAR(p->y.value(), 0.0, 1e-8);
            const double dx = end->x.value() - p->x.value();
            const double dy = end->y.value() - p->y.value();
            EXPECT_NEAR(std::hypot(dx, dy), 5.0, 1e-6);
            if (constraint == Constraint::Parallel) {
                EXPECT_NEAR(dy, 0.0, 1e-6);
            } else if (constraint == Constraint::Perpendicular) {
                EXPECT_NEAR(dx, 0.0, 1e-6);
            } else {
                EXPECT_NEAR(std::acos(dx / std::hypot(dx, dy)), targetAngle, 1e-6);
            }
        }
    }
}

TEST(DCMManagerSharedPointSolveTest, SeveralLinesCanSatisfyDifferentConstraintsAtOnePoint) {
    DCMManager manager;
    const auto a = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto shared = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    const auto parallelEnd = manager.addFigure(FigureDescriptor::point(13.0, 4.0));
    const auto perpendicularEnd = manager.addFigure(FigureDescriptor::point(12.0, 4.0));
    const auto angleEnd = manager.addFigure(FigureDescriptor::point(14.0, 3.0));
    const auto base = manager.addFigure(FigureDescriptor::line(a, shared));
    const auto parallel = manager.addFigure(FigureDescriptor::line(shared, parallelEnd));
    const auto perpendicular = manager.addFigure(FigureDescriptor::line(shared, perpendicularEnd));
    const auto angle = manager.addFigure(FigureDescriptor::line(shared, angleEnd));
    const double targetAngle = std::acos(-1.0) / 3.0;

    manager.addRequirement(RequirementDescriptor::fixLine(base));
    for (const auto endpoint : {parallelEnd, perpendicularEnd, angleEnd}) {
        manager.addRequirement(RequirementDescriptor::pointPointDist(shared, endpoint, 5.0));
    }
    manager.addRequirement(RequirementDescriptor::lineLineParallel(base, parallel));
    manager.addRequirement(RequirementDescriptor::lineLinePerpendicular(base, perpendicular));
    manager.addRequirement(RequirementDescriptor::lineLineAngle(base, angle, targetAngle));

    ASSERT_TRUE(manager.solve());
    const auto origin = manager.getFigure(shared);
    const auto p = manager.getFigure(parallelEnd);
    const auto q = manager.getFigure(perpendicularEnd);
    const auto r = manager.getFigure(angleEnd);
    ASSERT_TRUE(origin && p && q && r);
    const auto delta = [&](const std::optional<FigureDescriptor>& endpoint) {
        return std::pair{endpoint->x.value() - origin->x.value(),
                         endpoint->y.value() - origin->y.value()};
    };
    const auto [px, py] = delta(p);
    const auto [qx, qy] = delta(q);
    const auto [rx, ry] = delta(r);
    EXPECT_NEAR(py, 0.0, 1e-6);
    EXPECT_NEAR(qx, 0.0, 1e-6);
    EXPECT_NEAR(std::acos(rx / std::hypot(rx, ry)), targetAngle, 1e-6);
    for (const auto [dx, dy] : {std::pair{px, py}, std::pair{qx, qy}, std::pair{rx, ry}}) {
        EXPECT_NEAR(std::hypot(dx, dy), 5.0, 1e-6);
    }
    EXPECT_NEAR(origin->x.value(), 10.0, 1e-8);
    EXPECT_NEAR(origin->y.value(), 0.0, 1e-8);
}

TEST(DCMManagerSharedPointSolveTest, JacobianIncludesEveryUseOfSharedCoordinates) {
    DCMManager manager;
    const auto a = manager.addFigure(FigureDescriptor::point(1.0, 2.0));
    const auto shared = manager.addFigure(FigureDescriptor::point(4.0, 3.0));
    const auto c = manager.addFigure(FigureDescriptor::point(6.0, 7.0));
    const auto first = manager.addFigure(FigureDescriptor::line(a, shared));
    const auto second = manager.addFigure(FigureDescriptor::line(shared, c));
    manager.addRequirement(RequirementDescriptor::lineLineParallel(first, second));
    manager.addRequirement(RequirementDescriptor::lineLinePerpendicular(first, second));
    manager.addRequirement(RequirementDescriptor::lineLineAngle(first, second, 0.75));

    const auto& system = manager.getRequirementSystem();
    const auto vars = system.getAllVars();
    auto* point = manager.storage().get<Figures::Point2D>(shared);
    ASSERT_NE(point, nullptr);
    const auto jacobian = Eigen::MatrixXd(system.J());
    ASSERT_EQ(jacobian.rows(), 3);
    for (double* coordinate : {point->ptrX(), point->ptrY()}) {
        const auto it = std::find(vars.begin(), vars.end(), coordinate);
        ASSERT_NE(it, vars.end());
        const auto column = static_cast<Eigen::Index>(std::distance(vars.begin(), it));
        const double original = *coordinate;
        *coordinate = original + 1e-6;
        const auto forward = system.residuals();
        *coordinate = original - 1e-6;
        const auto backward = system.residuals();
        *coordinate = original;
        for (Eigen::Index row = 0; row < jacobian.rows(); ++row) {
            EXPECT_NEAR(jacobian(row, column), (forward[row] - backward[row]) / 2e-6, 2e-5);
        }
    }
}

TEST_F(DCMManagerSolveTest, GlobalSolve_PointPointDist) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(3.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    double dx = d2->x.value() - d1->x.value();
    double dy = d2->y.value() - d1->y.value();
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 5.0, 0.1);
}

TEST_F(DCMManagerSolveTest, DifferentWeightsChooseWeightedDistanceCompromise) {
    const auto fixed = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto moving = manager.addFigure(FigureDescriptor::point(7.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(fixed));
    auto nearDistance = RequirementDescriptor::pointPointDist(fixed, moving, 5.0);
    auto farDistance = RequirementDescriptor::pointPointDist(fixed, moving, 10.0);
    nearDistance.weight = 3.0;
    farDistance.weight = 1.0;
    const auto nearId = manager.addRequirement(nearDistance);
    const auto farId = manager.addRequirement(farDistance);

    const auto& system = manager.getRequirementSystem();
    EXPECT_NEAR(system.residuals()[2], 6.0, 1e-12);
    EXPECT_NEAR(system.residuals()[3], -3.0, 1e-12);
    const auto jacobian = Eigen::MatrixXd(system.J());
    const auto vars = system.getAllVars();
    auto* point = manager.storage().get<Figures::Point2D>(moving);
    ASSERT_NE(point, nullptr);
    const auto it = std::find(vars.begin(), vars.end(), point->ptrX());
    ASSERT_NE(it, vars.end());
    const auto column = static_cast<Eigen::Index>(std::distance(vars.begin(), it));
    EXPECT_NEAR(jacobian(2, column), 3.0, 1e-12);
    EXPECT_NEAR(jacobian(3, column), 1.0, 1e-12);

    EXPECT_FALSE(manager.solve()); // The two target distances conflict.
    EXPECT_NEAR(manager.getFigure(moving)->x.value(), 5.5, 1e-4);
    EXPECT_NEAR(manager.getFigure(moving)->y.value(), 0.0, 1e-6);

    manager.updateRequirementWeight(nearId, 1.0);
    manager.updateRequirementWeight(farId, 3.0);
    EXPECT_DOUBLE_EQ(manager.getRequirement(nearId)->weight, 1.0);
    EXPECT_DOUBLE_EQ(manager.getRequirement(farId)->weight, 3.0);
    EXPECT_FALSE(manager.solve());
    EXPECT_NEAR(manager.getFigure(moving)->x.value(), 9.5, 1e-4);

    manager.setSolveMode(SolveMode::LOCAL);
    manager.updateRequirementWeight(nearId, 3.0);
    manager.updateRequirementWeight(farId, 1.0);
    const auto component = manager.getComponentForFigure(moving);
    ASSERT_TRUE(component.has_value());
    EXPECT_FALSE(manager.solve(*component));
    EXPECT_NEAR(manager.getFigure(moving)->x.value(), 5.5, 1e-4);
}

TEST_F(DCMManagerSolveTest, CompatibleWeightedConstraintsStillSolve) {
    const auto fixed = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto moving = manager.addFigure(FigureDescriptor::point(7.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(fixed));
    auto first = RequirementDescriptor::pointPointDist(fixed, moving, 5.0);
    auto second = RequirementDescriptor::pointPointDist(fixed, moving, 5.0);
    first.weight = 2.0;
    second.weight = 5.0;
    manager.addRequirement(first);
    manager.addRequirement(second);

    EXPECT_TRUE(manager.solve());
    EXPECT_NEAR(manager.getFigure(moving)->x.value(), 5.0, 1e-6);
    EXPECT_LT(manager.getRequirementSystem().residuals().cwiseAbs().maxCoeff(), 1e-6);
}

TEST_F(DCMManagerSolveTest, ZeroWeightDisablesOrdinaryRequirement) {
    const auto fixed = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto moving = manager.addFigure(FigureDescriptor::point(7.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(fixed));
    auto active = RequirementDescriptor::pointPointDist(fixed, moving, 5.0);
    auto inactive = RequirementDescriptor::pointPointDist(fixed, moving, 10.0);
    active.weight = 2.0;
    inactive.weight = 0.0;
    manager.addRequirement(active);
    manager.addRequirement(inactive);

    EXPECT_EQ(manager.getRequirementSystem().diagnose(), SystemStatus::UNDER_CONSTRAINED);
    EXPECT_TRUE(manager.solve());
    EXPECT_NEAR(manager.getFigure(moving)->x.value(), 5.0, 1e-6);
    const auto residuals = manager.getRequirementSystem().residuals();
    EXPECT_NEAR(residuals[2], 0.0, 1e-6);
    EXPECT_DOUBLE_EQ(residuals[3], 0.0);
}

TEST_F(DCMManagerSolveTest, AllZeroWeightRequirementsDiagnoseAsEmpty) {
    const auto first = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto second = manager.addFigure(FigureDescriptor::point(7.0, 0.0));
    auto distance = RequirementDescriptor::pointPointDist(first, second, 5.0);
    distance.weight = 0.0;
    manager.addRequirement(distance);

    EXPECT_EQ(manager.getRequirementSystem().diagnose(), SystemStatus::EMPTY);
    EXPECT_TRUE(manager.solve());
}

TEST_F(DCMManagerSolveTest, RepeatedSolveUsesChangedPointCoordinates) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));
    const auto& system = manager.getRequirementSystem();

    EXPECT_TRUE(manager.solve());
    const Eigen::MatrixXd initialJ = Eigen::MatrixXd(system.J());
    EXPECT_NEAR(initialJ(0, 2), 1.0, 1e-12);

    manager.updatePoint(PointUpdateDescriptor(p2, 0.0, 8.0));
    EXPECT_NEAR(system.residuals()[0], 3.0, 1e-12);
    const Eigen::MatrixXd movedJ = Eigen::MatrixXd(system.J());
    EXPECT_FALSE(initialJ.isApprox(movedJ));
    EXPECT_NEAR(movedJ(0, 3), 1.0, 1e-12);

    EXPECT_TRUE(manager.solve());
    EXPECT_NEAR(system.residuals()[0], 0.0, 1e-6);
    const auto d1 = manager.getFigure(p1);
    const auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());
    EXPECT_NEAR(std::hypot(d2->x.value() - d1->x.value(),
                           d2->y.value() - d1->y.value()), 5.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, SolveRejectsIncompatibleDistanceBetweenFixedPoints) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    const auto component = manager.getComponentForFigure(p1);
    ASSERT_TRUE(component.has_value());

    for (const auto mode : {SolveMode::GLOBAL, SolveMode::LOCAL, SolveMode::DRAG}) {
        manager.setSolveMode(mode);
        EXPECT_FALSE(manager.solve(*component));
    }

    const auto d1 = manager.getFigure(p1);
    const auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());
    EXPECT_DOUBLE_EQ(d1->x.value(), 0.0);
    EXPECT_DOUBLE_EQ(d1->y.value(), 0.0);
    EXPECT_DOUBLE_EQ(d2->x.value(), 5.0);
    EXPECT_DOUBLE_EQ(d2->y.value(), 0.0);
}

TEST_F(DCMManagerSolveTest, SolveRejectsNonzeroResidualAtZeroGradient) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    const auto& system = manager.getRequirementSystem();
    EXPECT_DOUBLE_EQ(system.residuals()[0], -10.0);
    EXPECT_DOUBLE_EQ((system.J().transpose() * system.residuals()).norm(), 0.0);

    EXPECT_FALSE(manager.solve());
    EXPECT_DOUBLE_EQ(system.residuals()[0], -10.0);
}

TEST_F(DCMManagerSolveTest, SolveAcceptsSatisfiedDistanceBetweenFixedPoints) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));

    EXPECT_TRUE(manager.solve());
}

TEST_F(DCMManagerSolveTest, SolveChecksEachResidualAgainstSpecifiedTolerance) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.00005));

    EXPECT_FALSE(manager.solve());
    EXPECT_TRUE(manager.solve(std::nullopt, 1e-4));
    EXPECT_FALSE(manager.solve(std::nullopt, 1e-6));
}

TEST_F(DCMManagerSolveTest, SolveRejectsInvalidResidualTolerance) {
    EXPECT_THROW(manager.solve(std::nullopt, -1.0), std::invalid_argument);
    EXPECT_THROW(manager.solve(std::nullopt, std::numeric_limits<double>::infinity()), std::invalid_argument);
    EXPECT_THROW(manager.solve(std::nullopt, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST_F(DCMManagerSolveTest, SolveRejectsNonfiniteResidual) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(-1e200, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(1e200, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));

    EXPECT_FALSE(manager.solve());
}

TEST_F(DCMManagerSolveTest, RejectedNonfiniteParameterPreservesValidSolveState) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    const auto distance = manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));
    ASSERT_TRUE(manager.solve());
    EXPECT_THROW(manager.updateRequirementParam(distance, std::numeric_limits<double>::quiet_NaN()),
                 std::invalid_argument);
    EXPECT_DOUBLE_EQ(*manager.getRequirement(distance)->param, 5.0);
    EXPECT_TRUE(manager.solve());
}

TEST_F(DCMManagerSolveTest, SolveRejectsConflictingFixedTargetsOnAliasedPoints) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(p1, p2));

    EXPECT_FALSE(manager.solve());
}

TEST_F(DCMManagerSolveTest, LocalSolveOnlyChecksSelectedComponentResiduals) {
    const auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    const auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    const auto p3 = manager.addFigure(FigureDescriptor::point(100.0, 100.0));
    const auto p4 = manager.addFigure(FigureDescriptor::point(105.0, 100.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p3));
    manager.addRequirement(RequirementDescriptor::fixPoint(p4));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p3, p4, 5.0));
    const auto component = manager.getComponentForFigure(p3);
    ASSERT_TRUE(component.has_value());

    manager.setSolveMode(SolveMode::LOCAL);
    EXPECT_TRUE(manager.solve(*component));
    manager.setSolveMode(SolveMode::GLOBAL);
    EXPECT_FALSE(manager.solve());
}

TEST_F(DCMManagerSolveTest, GlobalSolve_Horizontal) {
    auto line = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 5.0, 3.0));
    auto lineDesc = manager.getFigure(line);
    ASSERT_TRUE(lineDesc.has_value());
    auto p1 = lineDesc->pointIds[0];
    auto p2 = lineDesc->pointIds[1];

    manager.addRequirement(RequirementDescriptor::horizontal(line));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    EXPECT_NEAR(d1->y.value(), d2->y.value(), 0.1);
}

TEST_F(DCMManagerSolveTest, LocalSolve_SingleComponent) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(1.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(100.0, 100.0));
    auto p4 = manager.addFigure(FigureDescriptor::point(103.0, 100.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p3, p4, 20.0));

    EXPECT_EQ(manager.getComponentCount(), 2);

    auto comp12 = manager.getComponentForFigure(p1);
    ASSERT_TRUE(comp12.has_value());

    auto d3before = manager.getFigure(p3);
    auto d4before = manager.getFigure(p4);

    manager.setSolveMode(SolveMode::LOCAL);
    bool converged = manager.solve(comp12.value());
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    double dx = d2->x.value() - d1->x.value();
    double dy = d2->y.value() - d1->y.value();
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 10.0, 0.1);

    auto d3after = manager.getFigure(p3);
    auto d4after = manager.getFigure(p4);
    ASSERT_TRUE(d3after.has_value() && d4after.has_value());
    EXPECT_DOUBLE_EQ(d3after->x.value(), d3before->x.value());
    EXPECT_DOUBLE_EQ(d3after->y.value(), d3before->y.value());
    EXPECT_DOUBLE_EQ(d4after->x.value(), d4before->x.value());
    EXPECT_DOUBLE_EQ(d4after->y.value(), d4before->y.value());
}

TEST_F(DCMManagerSolveTest, LocalSolve_AfterRequirementRemovalIncludesLineAndPointConstraints) {
    const auto line = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 5.0, 3.0));
    const auto lineDesc = manager.getFigure(line);
    ASSERT_TRUE(lineDesc.has_value());
    ASSERT_EQ(lineDesc->pointIds.size(), 2U);
    const auto p1 = lineDesc->pointIds[0];
    const auto p2 = lineDesc->pointIds[1];
    const auto p3 = manager.addFigure(FigureDescriptor::point(100.0, 100.0));
    const auto p4 = manager.addFigure(FigureDescriptor::point(103.0, 100.0));
    const auto horizontal = manager.addRequirement(RequirementDescriptor::horizontal(line));
    const auto fixedPoint = manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p3, p4, 20.0));
    const auto bridge = manager.addRequirement(RequirementDescriptor::pointPointDist(p2, p3, 100.0));
    ASSERT_EQ(manager.getComponentCount(), 1U);

    manager.removeRequirement(bridge);
    manager.setSolveMode(SolveMode::LOCAL);
    const auto component = manager.getComponentForFigure(p1);
    ASSERT_TRUE(component.has_value());
    EXPECT_TRUE(manager.solve(*component));

    const auto d1 = manager.getFigure(p1);
    const auto d2 = manager.getFigure(p2);
    const auto d3 = manager.getFigure(p3);
    const auto d4 = manager.getFigure(p4);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value() && d4.has_value());
    EXPECT_NEAR(d1->x.value(), 0.0, 1e-6);
    EXPECT_NEAR(d1->y.value(), 0.0, 1e-6);
    EXPECT_NEAR(d2->y.value(), d1->y.value(), 1e-6);
    EXPECT_DOUBLE_EQ(d3->x.value(), 100.0);
    EXPECT_DOUBLE_EQ(d3->y.value(), 100.0);
    EXPECT_DOUBLE_EQ(d4->x.value(), 103.0);
    EXPECT_DOUBLE_EQ(d4->y.value(), 100.0);
    EXPECT_EQ(manager.getComponentCount(), 2U);
    EXPECT_EQ(manager.getComponentForFigure(line), component);
    EXPECT_EQ(manager.getComponentForFigure(p2), component);
    EXPECT_EQ(manager.getRequirementsInComponent(*component), (std::vector<ID>{horizontal, fixedPoint}));
}

TEST_F(DCMManagerSolveTest, DragMode_AutoSolveOnUpdate) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p1, 2.0, 0.0));

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    double dx = d2->x.value() - d1->x.value();
    double dy = d2->y.value() - d1->y.value();
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 5.0, 0.5);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateFixedPoint_RevertsToFixedCoordinates) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 5.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p1, 2.0, 3.0));

    auto d1 = manager.getFigure(p1);
    ASSERT_TRUE(d1.has_value());
    EXPECT_NEAR(d1->x.value(), 0.0, 1e-6);
    EXPECT_NEAR(d1->y.value(), 0.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateFixedCircleRadius_Ignored) {
    auto circle = manager.addFigure(FigureDescriptor::circle(0.0, 0.0, 10.0));
    manager.addRequirement(RequirementDescriptor::fixCircle(circle));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updateCircle(CircleUpdateDescriptor(circle, 25.0));

    auto c = manager.getFigure(circle);
    ASSERT_TRUE(c.has_value());
    ASSERT_TRUE(c->radius.has_value());
    EXPECT_NEAR(c->radius.value(), 10.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateFixedCircleCenterAndRadius_Ignored) {
    auto circle = manager.addFigure(FigureDescriptor::circle(0.0, 0.0, 10.0));
    manager.addRequirement(RequirementDescriptor::fixCircle(circle));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updateCircle(CircleUpdateDescriptor(circle, 20.0, 30.0, 25.0));

    auto c = manager.getFigure(circle);
    ASSERT_TRUE(c.has_value());
    ASSERT_EQ(c->coords.size(), 2);
    ASSERT_TRUE(c->radius.has_value());
    EXPECT_NEAR(c->coords[0], 0.0, 1e-6);
    EXPECT_NEAR(c->coords[1], 0.0, 1e-6);
    EXPECT_NEAR(c->radius.value(), 10.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateLineLocksEndpointCoordinates) {
    auto line = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 10.0, 0.0));
    auto point = manager.addFigure(FigureDescriptor::point(5.0, 20.0));
    manager.addRequirement(RequirementDescriptor::pointOnLine(point, line));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updateLine(LineUpdateDescriptor(line, 0.0, 10.0, 10.0, 10.0));

    auto lineDesc = manager.getFigure(line);
    auto pointDesc = manager.getFigure(point);
    ASSERT_TRUE(lineDesc.has_value() && pointDesc.has_value());
    ASSERT_EQ(lineDesc->coords.size(), 4);
    EXPECT_NEAR(lineDesc->coords[0], 0.0, 1e-9);
    EXPECT_NEAR(lineDesc->coords[1], 10.0, 1e-9);
    EXPECT_NEAR(lineDesc->coords[2], 10.0, 1e-9);
    EXPECT_NEAR(lineDesc->coords[3], 10.0, 1e-9);
    EXPECT_NEAR(pointDesc->y.value(), 10.0, 1e-4);
}

TEST_F(DCMManagerSolveTest, FixPointUsesCoordinatesCapturedAtConstraintCreation) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(20.0, 0.0));

    manager.updatePoint(PointUpdateDescriptor(p1, 5.0, 5.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p1));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());
    EXPECT_NEAR(d1->x.value(), 5.0, 1e-6);
    EXPECT_NEAR(d1->y.value(), 5.0, 1e-6);

    const double dx = d2->x.value() - d1->x.value();
    const double dy = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 10.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, GlobalSolve_PointOnPointUsesSingleRepresentativePoint) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(14.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointOnPoint(p1, p2));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p3, 5.0));
    manager.setSolveMode(SolveMode::GLOBAL);

    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());
    EXPECT_NEAR(d1->x.value(), d2->x.value(), 1e-6);
    EXPECT_NEAR(d1->y.value(), d2->y.value(), 1e-6);

    const double dx = d3->x.value() - d2->x.value();
    const double dy = d3->y.value() - d2->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 5.0, 1e-4);
}

TEST_F(DCMManagerSolveTest, DragMode_UpdateAliasedPointKeepsDraggedCoordinates) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(20.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointOnPoint(p1, p2));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p2, p3, 5.0));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updatePoint(PointUpdateDescriptor(p1, 100.0, 0.0));

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());
    EXPECT_NEAR(d1->x.value(), 100.0, 1e-6);
    EXPECT_NEAR(d2->x.value(), 100.0, 1e-6);
    EXPECT_NEAR(d1->y.value(), 0.0, 1e-6);
    EXPECT_NEAR(d2->y.value(), 0.0, 1e-6);

    const double dx = d3->x.value() - d2->x.value();
    const double dy = d3->y.value() - d2->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 5.0, 1e-4);
}

TEST_F(DCMManagerSolveTest, DragMode_MovingOtherPointDoesNotMoveFixedPoint) {
    auto p1 = manager.addFigure(FigureDescriptor::point(5.85786437626905, 5.85786437626905));
    auto p2 = manager.addFigure(FigureDescriptor::point(20.0, 20.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 20.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(p2));
    manager.setSolveMode(SolveMode::DRAG);

    manager.updatePoint(PointUpdateDescriptor(p1, -10.0, -10.0));

    auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d2.has_value());
    EXPECT_NEAR(d2->x.value(), 20.0, 1e-9);
    EXPECT_NEAR(d2->y.value(), 20.0, 1e-9);
}

TEST_F(DCMManagerSolveTest, DragMode_FallbackWithoutLocksWhenNoDofLeft) {
    auto pFixed = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto pDrag = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(pFixed, pDrag, 10.0));
    manager.addRequirement(RequirementDescriptor::fixPoint(pFixed));
    manager.setSolveMode(SolveMode::DRAG);

    // Impossible to keep this exact drag position and the fixed distance simultaneously
    // if dragged vars are hard-locked. Solver should fallback and project to feasible state.
    manager.updatePoint(PointUpdateDescriptor(pDrag, 20.0, 0.0));

    auto dFixed = manager.getFigure(pFixed);
    auto dDrag = manager.getFigure(pDrag);
    ASSERT_TRUE(dFixed.has_value() && dDrag.has_value());
    EXPECT_NEAR(dFixed->x.value(), 0.0, 1e-9);
    EXPECT_NEAR(dFixed->y.value(), 0.0, 1e-9);

    const double dx = dDrag->x.value() - dFixed->x.value();
    const double dy = dDrag->y.value() - dFixed->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 10.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, SolveUsesUpdatedRequirementParameterAfterCacheInvalidation) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    const auto req = manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p1, 2.0, 0.0));

    manager.updateRequirementParam(req, 20.0);
    const auto component = manager.getComponentForFigure(p1);
    ASSERT_TRUE(component.has_value());

    manager.setSolveMode(SolveMode::LOCAL);
    const bool converged = manager.solve(component.value());
    EXPECT_TRUE(converged);

    const auto d1 = manager.getFigure(p1);
    const auto d2 = manager.getFigure(p2);
    ASSERT_TRUE(d1.has_value() && d2.has_value());

    const double dx = d2->x.value() - d1->x.value();
    const double dy = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 20.0, 1e-6);
}

TEST_F(DCMManagerSolveTest, DragMode_RemovedRequirementDoesNotAffectSeparatedComponent) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(10.0, 0.0));
    auto p3 = manager.addFigure(FigureDescriptor::point(20.0, 0.0));

    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));
    const auto req = manager.addRequirement(RequirementDescriptor::pointPointDist(p2, p3, 10.0));

    manager.setSolveMode(SolveMode::DRAG);
    manager.updatePoint(PointUpdateDescriptor(p2, 12.0, 0.0));

    const auto beforeRemovalP3 = manager.getFigure(p3);
    ASSERT_TRUE(beforeRemovalP3.has_value());

    manager.removeRequirement(req);
    EXPECT_EQ(manager.getComponentCount(), 2U);

    manager.updatePoint(PointUpdateDescriptor(p2, 30.0, 0.0));

    const auto d1 = manager.getFigure(p1);
    const auto d2 = manager.getFigure(p2);
    const auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());

    const double dx = d2->x.value() - d1->x.value();
    const double dy = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 10.0, 1e-6);
    EXPECT_NEAR(d3->x.value(), beforeRemovalP3->x.value(), 1e-9);
    EXPECT_NEAR(d3->y.value(), beforeRemovalP3->y.value(), 1e-9);
}

TEST_F(DCMManagerSolveTest, GlobalSolve_Rectangle) {
    auto l1 = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 100.0, 2.0));
    auto l2 = manager.addFigure(FigureDescriptor::line(100.0, 2.0, 102.0, 52.0));
    auto l3 = manager.addFigure(FigureDescriptor::line(102.0, 52.0, -1.0, 49.0));
    auto l4 = manager.addFigure(FigureDescriptor::line(-1.0, 49.0, 0.0, 0.0));

    auto dL1 = manager.getFigure(l1);
    auto dL2 = manager.getFigure(l2);
    ASSERT_TRUE(dL1.has_value() && dL2.has_value());
    auto p1 = dL1->pointIds[0];
    auto p2 = dL1->pointIds[1];
    auto p3 = dL2->pointIds[1];

    manager.addRequirement(RequirementDescriptor::horizontal(l1));
    manager.addRequirement(RequirementDescriptor::horizontal(l3));
    manager.addRequirement(RequirementDescriptor::vertical(l2));
    manager.addRequirement(RequirementDescriptor::vertical(l4));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 100.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p2, p3, 50.0));

    manager.setSolveMode(SolveMode::GLOBAL);
    bool converged = manager.solve();
    EXPECT_TRUE(converged);

    auto d1 = manager.getFigure(p1);
    auto d2 = manager.getFigure(p2);
    auto d3 = manager.getFigure(p3);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value());

    EXPECT_NEAR(d1->y.value(), d2->y.value(), 0.5);

    double dx12 = d2->x.value() - d1->x.value();
    double dy12 = d2->y.value() - d1->y.value();
    EXPECT_NEAR(std::sqrt(dx12 * dx12 + dy12 * dy12), 100.0, 1.0);

    double dx23 = d3->x.value() - d2->x.value();
    double dy23 = d3->y.value() - d2->y.value();
    EXPECT_NEAR(std::sqrt(dx23 * dx23 + dy23 * dy23), 50.0, 1.0);
}

TEST_F(DCMManagerSolveTest, GlobalSolve_ClosedPolylineViaPointOnPointAliases) {
    auto l1 = manager.addFigure(FigureDescriptor::line(0.0, 0.0, 10.0, 0.0));
    auto l2 = manager.addFigure(FigureDescriptor::line(12.0, 0.0, 12.0, 8.0));
    auto l3 = manager.addFigure(FigureDescriptor::line(12.0, 10.0, 0.0, 10.0));
    auto l4 = manager.addFigure(FigureDescriptor::line(-2.0, 10.0, -2.0, 0.0));

    auto d1 = manager.getFigure(l1);
    auto d2 = manager.getFigure(l2);
    auto d3 = manager.getFigure(l3);
    auto d4 = manager.getFigure(l4);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value() && d4.has_value());

    manager.addRequirement(RequirementDescriptor::pointOnPoint(d1->pointIds[1], d2->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(d2->pointIds[1], d3->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(d3->pointIds[1], d4->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::pointOnPoint(d4->pointIds[1], d1->pointIds[0]));
    manager.addRequirement(RequirementDescriptor::horizontal(l1));
    manager.addRequirement(RequirementDescriptor::vertical(l2));
    manager.addRequirement(RequirementDescriptor::horizontal(l3));
    manager.addRequirement(RequirementDescriptor::vertical(l4));

    manager.setSolveMode(SolveMode::GLOBAL);
    const bool converged = manager.solve();
    EXPECT_TRUE(converged);

    d1 = manager.getFigure(l1);
    d2 = manager.getFigure(l2);
    d3 = manager.getFigure(l3);
    d4 = manager.getFigure(l4);
    ASSERT_TRUE(d1.has_value() && d2.has_value() && d3.has_value() && d4.has_value());

    auto p11 = manager.getFigure(d1->pointIds[0]);
    auto p12 = manager.getFigure(d1->pointIds[1]);
    auto p21 = manager.getFigure(d2->pointIds[0]);
    auto p22 = manager.getFigure(d2->pointIds[1]);
    auto p31 = manager.getFigure(d3->pointIds[0]);
    auto p32 = manager.getFigure(d3->pointIds[1]);
    auto p41 = manager.getFigure(d4->pointIds[0]);
    auto p42 = manager.getFigure(d4->pointIds[1]);
    ASSERT_TRUE(p11.has_value() && p12.has_value() && p21.has_value() && p22.has_value()
        && p31.has_value() && p32.has_value() && p41.has_value() && p42.has_value());

    EXPECT_NEAR(p12->x.value(), p21->x.value(), 1e-6);
    EXPECT_NEAR(p12->y.value(), p21->y.value(), 1e-6);
    EXPECT_NEAR(p22->x.value(), p31->x.value(), 1e-6);
    EXPECT_NEAR(p22->y.value(), p31->y.value(), 1e-6);
    EXPECT_NEAR(p32->x.value(), p41->x.value(), 1e-6);
    EXPECT_NEAR(p32->y.value(), p41->y.value(), 1e-6);
    EXPECT_NEAR(p42->x.value(), p11->x.value(), 1e-6);
    EXPECT_NEAR(p42->y.value(), p11->y.value(), 1e-6);
}

TEST_F(DCMManagerSolveTest, SolveEmptySystem) {
    manager.setSolveMode(SolveMode::GLOBAL);
    EXPECT_TRUE(manager.solve());
}

TEST_F(DCMManagerSolveTest, LocalSolve_ThrowsWithoutComponentId) {
    auto p1 = manager.addFigure(FigureDescriptor::point(0.0, 0.0));
    auto p2 = manager.addFigure(FigureDescriptor::point(5.0, 0.0));
    manager.addRequirement(RequirementDescriptor::pointPointDist(p1, p2, 10.0));

    manager.setSolveMode(SolveMode::LOCAL);
    EXPECT_THROW(manager.solve(), std::runtime_error);
}
