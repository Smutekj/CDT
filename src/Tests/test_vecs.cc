#include <gtest/gtest.h>

#include "../../include/Vector2.h"

TEST(TestVector2i, BasicAssertions)
{
    using namespace cdt;

    Vector2i v1(1, 3);
    EXPECT_EQ(v1.x, 1);
    EXPECT_EQ(v1.y, 3);

    auto v2 = v1;

    EXPECT_TRUE(v2.y == v1.y && v2.x == v1.x);
    EXPECT_TRUE(v2 == v1);

    auto v3 = v2 + v1;
    EXPECT_TRUE(v3 == Vector2i(2, 6));

    auto v4 = v2 - v1;
    EXPECT_TRUE(v4 == Vector2i(0, 0));

    auto v5 = v1 * 3;
    EXPECT_TRUE(v5 == Vector2i(3, 9));

    v5 = 3 * v1;
    EXPECT_TRUE(v5 == Vector2i(3, 9));
}

TEST(TestVector2f, BasicAssertions)
{
    using namespace cdt;

    Vector2f v1(1, 3);
    EXPECT_FLOAT_EQ(v1.x, 1);
    EXPECT_FLOAT_EQ(v1.y, 3);

    auto v2 = v1;

    EXPECT_TRUE(v2.y == v1.y && v2.x == v1.x);

    auto v3 = v2 + v1;
    EXPECT_FLOAT_EQ(v3.x, 2.f);
    EXPECT_FLOAT_EQ(v3.y, 6.f);

    auto v4 = v2 - v1;
    EXPECT_TRUE(v4 == Vector2f(0, 0));

    auto v5 = v1 * 3;
    EXPECT_TRUE(v5 == Vector2f(3, 9));

    v5 = 3 * v1;
    EXPECT_TRUE(v5 == Vector2f(3, 9));
}

TEST(VecCasting, BasicAssertions)
{
    using namespace cdt;

    Vector2i v1(1, 3);

    auto v1f = asFloat(v1);

    EXPECT_FLOAT_EQ(v1f.x, v1.x);
    EXPECT_FLOAT_EQ(v1f.y, v1.y);
}


