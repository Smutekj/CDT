#pragma once
#include <gtest/gtest.h>

#include "../Triangulation.h"

TEST(TestTriangulation, SuperTriangle) {

    using namespace cdt;

    Triangulation cdt;


    EXPECT_TRUE(cdt.m_vertices.empty());
    EXPECT_TRUE(cdt.m_triangles.empty());

    cdt.createSuperTriangle({10,10});

    EXPECT_TRUE(cdt.m_vertices.size() == 3);
    EXPECT_TRUE(cdt.m_triangles.size() == 1);

    EXPECT_TRUE(cdt.allAreDelaunay());

    cdt.insertVertex({5,5});
    cdt.insertVertex({5,8});
    cdt.insertVertex({8,3});
    cdt.insertVertex({2,4});

    EXPECT_TRUE(cdt.allAreDelaunay());
}



TEST(TestTriangulation, BoundingBox) {

    using namespace cdt;

    Triangulation cdt;

    cdt.createBoundary({10,10});

    cdt.insertVertex({5,5});
    cdt.insertVertex({5,8});
    cdt.insertVertex({8,3});
    cdt.insertVertex({2,4});

    EXPECT_TRUE(cdt.allAreDelaunay()); //! should this be true????

}


