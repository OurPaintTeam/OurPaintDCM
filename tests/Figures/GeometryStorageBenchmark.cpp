/**
 * Micro-benchmarks for GeometryStorage (STEP 10).
 * Run: GeometryStorageBenchmark --gtest_filter=GeometryStorageBench.*
 * Timings go to stderr; tests always pass if invariants hold.
 */
#include <gtest/gtest.h>
#include "GeometryStorage.h"

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

using namespace OurPaintDCM::Figures;
using namespace OurPaintDCM::Utils;

namespace {

using Clock = std::chrono::steady_clock;

void reportMs(std::string_view label, Clock::duration d) {
    const double ms =
        std::chrono::duration<double, std::milli>(d).count();
    std::cerr << "[GeometryStorageBench] " << label << ": " << ms << " ms\n";
}

} // namespace

TEST(GeometryStorageBench, CreateManyPoints) {
    constexpr int N = 50'000;
    GeometryStorage storage;
    const auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
        (void)storage.createPoint(static_cast<double>(i), static_cast<double>(i * 2));
    }
    reportMs("createPoint x " + std::to_string(N), Clock::now() - t0);
    EXPECT_EQ(static_cast<int>(storage.pointCount()), N);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageBench, LookupManyTimes) {
    constexpr int N = 10'000;
    constexpr int lookups = 200'000;
    GeometryStorage storage;
    std::vector<ID> ids;
    ids.reserve(N);
    for (int i = 0; i < N; ++i) {
        ids.push_back(storage.createPoint(static_cast<double>(i), 0.0));
    }
    const auto t0 = Clock::now();
    volatile double sink = 0;
    for (int k = 0; k < lookups; ++k) {
        const ID id = ids[static_cast<std::size_t>(k % N)];
        const Point2D* p = storage.get<Point2D>(id);
        sink += p->x();
    }
    reportMs("get<Point2D> x " + std::to_string(lookups), Clock::now() - t0);
    EXPECT_GT(sink, 0.0);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageBench, GetDependentsHub) {
    constexpr int spokes = 20'000;
    GeometryStorage storage;
    const ID hub = storage.createPoint(0.0, 0.0);
    const auto tCreate0 = Clock::now();
    for (int i = 0; i < spokes; ++i) {
        const ID rim = storage.createPoint(static_cast<double>(i), 1.0);
        ASSERT_TRUE(storage.createLine(hub, rim).has_value());
    }
    reportMs("star createLine x " + std::to_string(spokes), Clock::now() - tCreate0);

    const auto tDep0 = Clock::now();
    std::vector<ID> deps = storage.getDependents(hub);
    reportMs("getDependents(hub) degree=" + std::to_string(deps.size()), Clock::now() - tDep0);

    EXPECT_EQ(deps.size(), static_cast<std::size_t>(spokes));
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageBench, RemoveCascadeFromHub) {
    constexpr int spokes = 5'000;
    GeometryStorage storage;
    const ID hub = storage.createPoint(0.0, 0.0);
    for (int i = 0; i < spokes; ++i) {
        const ID rim = storage.createPoint(static_cast<double>(i), 1.0);
        ASSERT_TRUE(storage.createLine(hub, rim).has_value());
    }
    const auto t0 = Clock::now();
    EXPECT_EQ(storage.remove(hub, true), RemoveResult::Ok);
    reportMs("remove(hub,cascade) after " + std::to_string(spokes) + " lines", Clock::now() - t0);
    EXPECT_EQ(storage.lineCount(), 0u);
    EXPECT_FALSE(storage.contains(hub));
    EXPECT_EQ(storage.pointCount(), static_cast<std::size_t>(spokes));
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageBench, SubgraphBuild) {
    constexpr int chain = 2'000;
    GeometryStorage storage;
    ID prev = storage.createPoint(0.0, 0.0);
    for (int i = 1; i <= chain; ++i) {
        const ID cur = storage.createPoint(static_cast<double>(i), 0.0);
        ASSERT_TRUE(storage.createLine(prev, cur).has_value());
        prev = cur;
    }
    const ID mid = prev;
    const auto t0 = Clock::now();
    auto gOpt = storage.buildObjectSubgraph(mid);
    ASSERT_TRUE(gOpt.has_value());
    reportMs("buildObjectSubgraph(chain n=" + std::to_string(chain + 1) + ")", Clock::now() - t0);
    EXPECT_FALSE(gOpt->vertexCount() == 0);
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}

TEST(GeometryStorageBench, ReuseSlotsStress) {
    constexpr int cycles = 2'000;
    GeometryStorage storage;
    const auto t0 = Clock::now();
    for (int c = 0; c < cycles; ++c) {
        const ID a = storage.createPoint(0.0, 0.0);
        const ID b = storage.createPoint(1.0, 0.0);
        ASSERT_TRUE(storage.createLine(a, b).has_value());
        EXPECT_EQ(storage.remove(a, true), RemoveResult::Ok);
        EXPECT_EQ(storage.remove(b, false), RemoveResult::Ok);
    }
    reportMs("create/remove cycle x " + std::to_string(cycles), Clock::now() - t0);
    EXPECT_TRUE(storage.empty());
#ifndef NDEBUG
    EXPECT_TRUE(storage.validate());
#endif
}
