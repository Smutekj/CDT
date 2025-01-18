#pragma once
#include <gtest/gtest.h>

#include <Triangulation.h>

namespace cdt{

TEST(TestTriangulation, SuperTriangle)
{

    Triangulation triangulation;

    EXPECT_TRUE(triangulation.m_vertices.empty());
    EXPECT_TRUE(triangulation.m_triangles.empty());

    triangulation.createSuperTriangle({10, 10});

    EXPECT_TRUE(triangulation.m_vertices.size() == 3);
    EXPECT_TRUE(triangulation.m_triangles.size() == 1);

    EXPECT_TRUE(triangulation.allAreDelaunay());

    triangulation.insertVertex({5, 5});
    triangulation.insertVertex({5, 8});
    triangulation.insertVertex({8, 3});
    triangulation.insertVertex({2, 4});

    EXPECT_TRUE(triangulation.allAreDelaunay());
}



TEST(TestTriangulation, BoundingBox)
{

    Triangulation cdt;

    cdt.createBoundary({10, 10});

    cdt.insertVertex({5, 5});
    cdt.insertVertex({5, 8});
    cdt.insertVertex({8, 3});
    cdt.insertVertex({2, 4});

    EXPECT_TRUE(cdt.allAreDelaunay()); //! should this be true????
}

TEST(TestTriangulation, InsertionOnExisting)
{

    Triangulation cdt;

    cdt.createBoundary({10, 10});

    cdt.insertVertex({5, 5});
    cdt.insertVertex({5, 8});
    cdt.insertVertex({8, 3});
    cdt.insertVertex({2, 4});

    auto insertion_data = cdt.insertVertexAndGetData({5, 5});
    EXPECT_TRUE(insertion_data.overlapping_vertex == 4);
    EXPECT_TRUE(insertion_data.overlapping_edge.from == -1 && insertion_data.overlapping_edge.to == -1);
}

TEST(TestTriangulation, InsertionOnEdge)
{

    Triangulation cdt;

    cdt.createBoundary({10, 10});

    cdt.insertVertex({5, 5});
    cdt.insertVertex({5, 7});
    cdt.insertConstraint({4, 5});
    auto insertion_data = cdt.insertVertexAndGetData({5, 6});
    EXPECT_TRUE(insertion_data.overlapping_vertex == -1u) << "overlapping vertex should be -1 because it was not inserted on an existing one";
    EXPECT_TRUE((insertion_data.overlapping_edge.from == 5  && insertion_data.overlapping_edge.to == 4 ) ||
                (insertion_data.overlapping_edge.from == 4  && insertion_data.overlapping_edge.to == 5)) << "not inserted on edge";
}



class TriangulationTest : public ::testing::Test
{

protected:
    int testSomething(Triangulation<Vector2i> &cdt)
    {

        EXPECT_EQ(cdt.m_last_found, 0);
        return cdt.m_last_found;
    }
};

TEST_F(TriangulationTest, Test1)
{

    Triangulation<Vector2i> t;
    int x = testSomething(t);


}

}