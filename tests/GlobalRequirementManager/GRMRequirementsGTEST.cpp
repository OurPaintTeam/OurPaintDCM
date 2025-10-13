#include <gtest/gtest.h>
#include "GlobalRequirementManager.h"
#include "RequirementData.h"
#include "FigureData.h"

using namespace OurPaintDCM;
using namespace OurPaintDCM::Utils;

namespace {
struct DummyRequirement : Requirements::Requirement {
    double param;
    DummyRequirement(double p = 1.0) : Requirement(RequirementType::ET_HORIZONTAL), param(p) {}
    double getParam() const noexcept override { return param; }
    ErrorFunction* toFunction() override {
        return new ErrorFunction(std::vector<Variable*>(),1);
    }
};
}

// --- TESTS ---

TEST(GlobalRequirementManagerTest, AddRequirementThrowsIfNullptr) {
    GlobalRequirementManager mgr;
    RequirementData req;
    req.requirement = nullptr;
    EXPECT_THROW(mgr.addRequirement(req), std::invalid_argument);
}

TEST(GlobalRequirementManagerTest, AddRequirementStoresAndCreatesGraphEdges) {
    GlobalRequirementManager mgr;
    DummyRequirement reqObj(2.0);

    FigureData fig(FigureType::ET_LINE, ID(1), {1.0, 2.0}, {ID(2), ID(3)});
    RequirementData req(RequirementType::ET_HORIZONTAL, &reqObj, {fig});

    EXPECT_NO_THROW(mgr.addRequirement(req));
}

TEST(GlobalRequirementManagerTest, RemoveRequirementThrowsIfNotExist) {
    GlobalRequirementManager mgr;
    EXPECT_THROW(mgr.removeRequirement(ID(5)), std::invalid_argument);
}

TEST(GlobalRequirementManagerTest, AddAndRemoveRequirementWorks) {
    GlobalRequirementManager mgr;
    DummyRequirement reqObj(3.14);

    FigureData fig(FigureType::ET_CIRCLE, ID(10), {0.5}, {});
    RequirementData req(RequirementType::ET_HORIZONTAL, &reqObj, {fig});
    mgr.addRequirement(req);

    // ID назначается через генератор, получаем первый добавленный
    auto stored = mgr.getRequirement(ID(1));
    EXPECT_DOUBLE_EQ(stored.param, 3.14);

    EXPECT_NO_THROW(mgr.removeRequirement(stored.id));
}

TEST(GlobalRequirementManagerTest, RemoveAllRequirementsClearsData) {
    GlobalRequirementManager mgr;
    DummyRequirement reqObj(1.0);
    FigureData f(FigureType::ET_LINE, ID(11), {1.1}, {});
    RequirementData req(RequirementType::ET_HORIZONTAL, &reqObj, {f});
    mgr.addRequirement(req);

    EXPECT_NO_THROW(mgr.removeAllRequirements());
    // После очистки getRequirement должен ничего не вернуть
    EXPECT_THROW(mgr.getRequirement(ID(0)), std::out_of_range);
}

TEST(GlobalRequirementManagerTest, GetRequirementReturnsCopy) {
    GlobalRequirementManager mgr;
    DummyRequirement reqObj(2.71);
    FigureData f(FigureType::ET_LINE, ID(7), {2.0}, {});
    RequirementData req(RequirementType::ET_HORIZONTAL, &reqObj, {f});
    mgr.addRequirement(req);

    auto copy = mgr.getRequirement(ID(1));
    EXPECT_NE(&copy, &req);
}
