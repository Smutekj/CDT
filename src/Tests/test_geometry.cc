#include <gtest/gtest.h>

#include "../Vector2.h"

TEST(TestBasicGeometry, Dot)
{
    using namespace cdt;

    Vector2i v1(3, 4);
    Vector2i v2(-1, 2);

    auto x = dot(v1, v2);

    EXPECT_EQ(x, 5); //! 3*(-1) + 2*4

    x = dot(v1, Vector2i{-4, 3}); //! perpendicular should be 0
    EXPECT_EQ(x, 0);

    Vector2f v1f(3, 4);
    Vector2f v2f(-1, 2);

    x = dot(v1, v2);
    EXPECT_FLOAT_EQ(x, 5);
}

TEST(TestBasicGeometry, Cross)
{
    using namespace cdt;

    //! points are perpendicular so cross product is just multiple of lengths
    Vector2f v1(1, 1);
    Vector2f v2(-1, 1);
    EXPECT_FLOAT_EQ(cross(v1, v2), 2);

    v1 = {1, 1};
    v2 = {-1, 2};
    EXPECT_FLOAT_EQ(cross(v1, v2), v1.x * v2.y - v1.y * v2.x);
}

TEST(TestBasicGeometry, Orientation)
{
    using namespace cdt;

    //! points are in counter clockwise order
    Vector2f v1(1, 0);
    Vector2f v2(0, 1);
    Vector2f v3(-1, -1);

    EXPECT_TRUE(orient(v1, v2, v3) > 0);
    EXPECT_TRUE(orient(v3, v2, v1) < 0);

    //! they are over 180 degrees
    v1 = {1, 0};
    v2 = {0, -1};
    v3 = {1, -1};
    EXPECT_TRUE(orient(v1, v2, v3) > 0);
}

TEST(TestBasicGeometry, Colinearity)
{
    using namespace cdt;

    //! points are in counter clockwise order
    Vector2f v1(1, 1);
    Vector2f v2(2, 2);
    Vector2f v3(3, 3);

    EXPECT_EQ(orient(v1, v2, v3), 0);
    EXPECT_EQ(orient(v3, v2, v1), 0);
}

TEST(TestSegmentsIntersection, BasicAssertions)
{
    using namespace cdt;

    //! intersection at point 1,1
    Vector2f v1s(0, 0);
    Vector2f v1e(2, 2);
    Vector2f v2s(0, 2);
    Vector2f v2e(2, 0);

    Vector2f intersection = {-1, -1};
    EXPECT_TRUE(segmentsIntersect(v1s, v1e, v2s, v2e, intersection));
    EXPECT_FLOAT_EQ(intersection.x, 1);
    EXPECT_FLOAT_EQ(intersection.y, 1);

    //! no intersection
    v1s = {0, 0};
    v1e = {2, 2};
    v2s = {0, 2};
    v2e = {2, 5};

    EXPECT_FALSE(segmentsIntersect(v1s, v1e, v2s, v2e, intersection));
    //! intersection should not have been changed!
    EXPECT_FLOAT_EQ(intersection.x, 1);
    EXPECT_FLOAT_EQ(intersection.y, 1);
}

TEST(TestSegmentsIntersectOrTouch, SegmentsTouchAtMidpoint)
{
    using namespace cdt;

    //! touch at point 1,0
    Vector2f v1s(0, 0);
    Vector2f v1e(2, 0);
    Vector2f v2s(1, 1);
    Vector2f v2e(1, 0);
    Vector2f intersection = {-1, -1};
    EXPECT_FALSE(segmentsIntersect(v1s, v1e, v2s, v2e, intersection));
    EXPECT_TRUE(segmentsIntersectOrTouch(v1s, v1e, v2s, v2e, intersection));
    EXPECT_FLOAT_EQ(intersection.x, 1);
    EXPECT_FLOAT_EQ(intersection.y, 0);
}

TEST(TestSegmentsIntersectOrTouch, SegmentsIntersect)
{
    using namespace cdt;

    //! intersection at point 1,1
    Vector2f v1s = {0, 0};
    Vector2f v1e = {2, 2};
    Vector2f v2s = {0, 2};
    Vector2f v2e = {2, 0};
    Vector2f intersection = {-1, -1};
    EXPECT_TRUE(segmentsIntersectOrTouch(v1s, v1e, v2s, v2e, intersection));
    EXPECT_FLOAT_EQ(intersection.x, 1);
    EXPECT_FLOAT_EQ(intersection.y, 1);
}

TEST(TestSegmentsIntersectOrTouch, SegmentsDontInteresct)
{
    using namespace cdt;

    //! no intersection
    Vector2f v1s = {0, 0};
    Vector2f v1e = {2, 2};
    Vector2f v2s = {0, 2};
    Vector2f v2e = {2, 5};
    Vector2f intersection = {-1, -1};
    EXPECT_FALSE(segmentsIntersectOrTouch(v1s, v1e, v2s, v2e, intersection));
    //! intersection should not have been changed!
    EXPECT_FLOAT_EQ(intersection.x, -1);
    EXPECT_FLOAT_EQ(intersection.y, -1);
}
TEST(TestSegmentsIntersectOrTouch, SegmentsTouchEndPoints)
{
    using namespace cdt;

    //! touch at end points 2,2
    Vector2f v1s = {0, 0};
    Vector2f v1e = {2, 2};
    Vector2f v2s = {15, 2};
    Vector2f v2e = {2, 2};
    Vector2f intersection = {-1, -1};
    EXPECT_TRUE(segmentsIntersectOrTouch(v1s, v1e, v2s, v2e, intersection));
    EXPECT_FLOAT_EQ(intersection.x, 2);
    EXPECT_FLOAT_EQ(intersection.y, 2);
}

TEST(TestSegmentsIntersectOrTouch, SegmentsTouchStartPoints)
{

    using namespace cdt;

    //! touch at start points 5,5
    Vector2f v1s = {5, 5};
    Vector2f v1e = {5, 2};
    Vector2f v2s = {5, 5};
    Vector2f v2e = {2, 13};
    Vector2f intersection = {-1, -1};
    EXPECT_TRUE(segmentsIntersectOrTouch(v1s, v1e, v2s, v2e, intersection));
    EXPECT_FLOAT_EQ(intersection.x, 5);
    EXPECT_FLOAT_EQ(intersection.y, 5);
}
