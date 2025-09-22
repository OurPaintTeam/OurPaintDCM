#include <gtest/gtest.h>
#include "IDGenerator.h"

using namespace OurPaintDCM::Utils;

TEST(IDGeneratorTest, DefaultConstructor) {
    IDGeneratorGTEST gen;
    EXPECT_EQ(gen.current().id, 1);
}

TEST(IDGeneratorTest, NextIDIncrements) {
    IDGeneratorGTEST gen;
    ID id1 = gen.nextID();
    ID id2 = gen.nextID();
    EXPECT_EQ(id1.id, 1);
    EXPECT_EQ(id2.id, 2);
    EXPECT_EQ(gen.current().id, 3);
}

TEST(IDGeneratorTest, Reset) {
    IDGeneratorGTEST gen;
    gen.nextID();
    gen.nextID();
    EXPECT_EQ(gen.current().id, 3);
    gen.reset();
    EXPECT_EQ(gen.current().id, 1);
    EXPECT_EQ(gen.nextID().id, 1);
}

TEST(IDGeneratorTest, SetCustomID) {
    IDGeneratorGTEST gen;
    gen.set(ID(100));
    EXPECT_EQ(gen.current().id, 100);
    EXPECT_EQ(gen.nextID().id, 100);
    EXPECT_EQ(gen.nextID().id, 101);
}

TEST(IDGeneratorTest, SequentialIDs) {
    IDGeneratorGTEST gen;
    std::vector<int> values;
    for (int i = 0; i < 5; ++i) {
        values.push_back(gen.nextID().id);
    }
    EXPECT_EQ(values, (std::vector<int>{1, 2, 3, 4, 5}));
}
