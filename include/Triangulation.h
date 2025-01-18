#ifndef BOIDS_TRIANGULATION_H
#define BOIDS_TRIANGULATION_H

#include <unordered_set>
#include <vector>
#include <array>
#include <memory>
#include <deque>
#include <stack>

#include "Grid.h"

//! \namespace cdt
namespace cdt
{

    using TriInd = unsigned int;
    using VertInd = unsigned int;

    //! \struct EdgeVInd
    //! \brief holding indices of two vertices in the triangulation
    struct EdgeVInd
    {
        VertInd from = -1;
        VertInd to = -1;

        EdgeVInd(VertInd from, VertInd to)
            : from(from), to(to) {}
        EdgeVInd() = default;

        bool operator==(const EdgeVInd &e) const
        {
            return (from == e.from and to == e.to) or (from == e.to and to == e.from);
        }
    };

    //! \class EdgeI
    //! \brief represents a segment using two vertices
    //! \tparam Vertex type of the vertex data held in the triangles
    template <class Vertex>
    struct EdgeI
    {
        Vertex from;
        Vertex t;

        EdgeI() = default;
        EdgeI(const Vertex &v1, const Vertex &v2)
            : from(v1), t(v2 - v1) {}

        float length() const { return norm(t); }
        Vertex to() const { return from + t; }
        bool operator==(const EdgeI &e) const { return e.from == from and e.t == t; }
    };

    //! \struct Triangle
    //! \tparam Vertex type of the vertex data held in the triangles
    //! \brief holds data relating to triangle. Ordering of vertices is counterclowise!
    template <class Vertex>
    struct Triangle
    {
        Vertex verts[3];                        //! vertex coordinates
        TriInd neighbours[3] = {-1u, -1u, -1u}; //! indices of neighbouring triangles
        std::array<bool, 3> is_constrained = {false, false, false};
        ; //! whether corresponding edge is constrained (is this needed here?)

        explicit Triangle() = default;

        cdt::Vector2f getCenter() const
        {
            return asFloat(verts[0] + verts[1] + verts[2]) / 3.f;
        }
    };

    //! \struct EdgeHash
    //! \brief function object used to convert an edge into a hash
    struct EdgeHash
    {
        template <class Vertex>
        std::size_t operator()(const EdgeI<Vertex> &e) const
        {
            return std::hash<VertInd>()(e.from.x) ^ std::hash<VertInd>()(e.t.x) ^ std::hash<VertInd>()(e.from.y) ^
                   std::hash<VertInd>()(e.t.y);
        }
    };

    //! \struct VertexInsertionData
    //! \brief holds data about how a vertex was inserted
    //! could either be inserted on an eixsting vertex,
    //! open space, or on an existing constrained edge
    struct VertexInsertionData
    {
        VertInd overlapping_vertex = -1;
        EdgeVInd overlapping_edge;
    };

    //! \class Triangulation
    //! \brief Holds all triangles, vertices and other data needed to construct and maintin a
    //! \brief constrained delaunay triangulation
    //! \tparam Vertex type of the vertex data held in the triangles
    template <class Vertex = cdt::Vector2<unsigned int>>
    class Triangulation
    {
    public:
        explicit Triangulation(Vertex box_size);
        Triangulation() = default;

        void reset();
        void createBoundaryAndSuperTriangle(cdt::Vector2i box_size);
        void createBoundary(cdt::Vector2i box_size);
        void createSuperTriangle(cdt::Vector2i box_size);

        TriInd findTriangle(cdt::Vector2f query_point, bool start_from_last_found = false);

        void insertVertex(const Vertex &v, bool = false);
        void insertVertexIntoSpace(const Vertex &v, TriInd, VertInd);
        VertexInsertionData insertVertexAndGetData(const Vertex &v, bool = false);
        VertexInsertionData insertVertexAndGetData(int vx, int vy, bool = false);

        void insertConstraint(const EdgeVInd edge);
        void insertVertices(const std::vector<Vertex> &verts);

        int indexOf(const Vertex &v, const Triangle<Vertex> &tri) const;
        int oppositeIndex(const TriInd np, const Triangle<Vertex> &tri);

        void dumpToFile(const std::string filename) const;

        void updateCellGrid();

        bool allAreDelaunay() const;

        //! \param query    the queried point
        //! \returns true if the \p query point lies within the rectangular boundary
        template <class VectorType>
        bool isWithinBoundary(const VectorType &query)
        {
            return query.x >= 0 && query.x <= m_boundary.x &&
                   query.y >= 0 && query.y <= m_boundary.y;
        }

    private:
        bool areCollinear(const Vertex &v1, const Vertex &v2, const Vertex &v3) const;
        bool liesBetween(const Vertex &v, const Vertex &v_left, const Vertex &v_right) const;

        VertInd findOverlappingVertex(const Vertex &new_vertex, const TriInd tri_ind) const;
        EdgeVInd findOverlappingEdge(const Vertex &new_vertex, const TriInd tri_ind) const;

        std::vector<EdgeVInd> findOverlappingEdges(const Vertex &vi, const Vertex &vj);
        std::vector<EdgeI<Vertex>> findOverlappingConstraints(const Vertex &vi, const Vertex &vj);

        TriInd findTriangleFromVertex(Vertex query_point, bool start_from_last_found = false);

        bool edgesIntersect(const EdgeVInd e1, const EdgeVInd e2) const noexcept;
        bool edgesIntersect(const EdgeI<Vertex> e1, const EdgeI<Vertex> e2) const noexcept;

        void insertVertexOnEdge(const Vertex &v, TriInd tri_ind_a, TriInd tri_ind_b, const EdgeI<Vertex> &edge);

        bool isConvex(const Vertex v1, const Vertex v2, const Vertex v3, const Vertex v4) const;

        void swapConnectingEdgeClockwise(const TriInd &tri_ind_a, const TriInd &tri_ind_b);
        void swapConnectingEdgeCounterClockwise(const TriInd &tri_ind_a, const TriInd &tri_ind_b);

        void fixDelaunayProperty(Vertex new_vertex, std::stack<std::pair<TriInd, TriInd>> &triangles_to_fix);

        void findIntersectingEdges(const EdgeVInd &e, std::deque<EdgeI<Vertex>> &intersected_edges,
                                   std::deque<TriInd> &intersected_tri_inds);

        bool needSwap(const Vertex &vp, const Vertex &v1, const Vertex &v2, const Vertex &v3) const;

        long long det(const Vertex &v1, const Vertex &v2, const Vertex &v3) const;
        bool isCounterClockwise(const Vertex &v_query, const Vertex &v1, const Vertex &v2) const;
        bool hasGoodOrientation(const Triangle<Vertex> &tri) const;
        bool allTrianglesValid() const;
        bool isDelaunay(const Triangle<Vertex> &tri) const;
        bool triangulationIsConsistent() const;

        int oppositeOfEdge(const Triangle<Vertex> &tri, const EdgeI<Vertex> &e) const;
        TriInd triangleOppositeOfEdge(const Triangle<Vertex> &tri, const EdgeI<Vertex> &edge) const;

        void updateIndsOfNeighbour(TriInd to_update, TriInd old_neighbour, TriInd new_neighbour);

        friend class TriangulationTest; //! controversial but IDGAF

    public:
        std::vector<Triangle<Vertex>> m_triangles;
        std::vector<Vertex> m_vertices;
        std::unordered_set<EdgeI<Vertex>, EdgeHash> m_fixed_edges;

        std::vector<std::array<VertInd, 3>> m_tri_ind2vert_inds;
        std::vector<std::array<bool, 3>> m_triedge_constrained;

    private:
        std::vector<TriInd> m_cell2tri_ind;
        cdt::Vector2i m_boundary;

        TriInd m_last_found = 0;      //! cached index of last found triangle (in a lot of cases new searched triangle is
                                      //! near previously found one)
        std::unique_ptr<Grid> m_grid; //! underlying grid that will be used for finding triangles containing query point
    };

    //! \brief used for finding orientation of \p p1 w.r.t. ( \p p3 - \p p2 )
    template <typename VectorType, typename VectorType2>
    inline float sign(const VectorType &p1, const VectorType2 &p2, const VectorType2 &p3)
    {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    // //! \brief used for finding orientation of \p p1 w.r.t. ( \p p3 - \p p2 )
    // // template <typename VectorType2>
    // inline float sign(const cdt::Vector2i &p1, const cdt::Vector2i &p2, const cdt::Vector2i &p3)
    // {
    //     return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    // }

    inline int next(const int ind_in_tri)
    {
        assert(ind_in_tri <= 2 && ind_in_tri >= 0);
        if (ind_in_tri == 2)
        {
            return 0;
        }
        return ind_in_tri + 1;
    }

    inline int prev(const int ind_in_tri)
    {
        assert(ind_in_tri <= 2 && ind_in_tri >= 0);
        if (ind_in_tri == 0)
        {
            return 2;
        }
        return ind_in_tri - 1;
    }

    template <class Vertex>
    inline int indInTriOf(const Triangle<Vertex> &tri, const TriInd neighbour)
    {
        if (tri.neighbours[0] == neighbour)
        {
            return 0;
        }
        else if (tri.neighbours[1] == neighbour)
        {
            return 1;
        }
        else if (tri.neighbours[2] == neighbour)
        {
            return 2;
        }
        //! neighbour not found
        return 3;
    }

    //! \brief checks if point \p r lies inside the triangle \p tri
    //! \tparam VecT
    //! \param r
    //! \param tri
    //! \returns true if the point lies inside the triangle
    template <typename VecT, class Vertex>
    inline bool isInTriangle(const VecT &r, const Triangle<Vertex> &tri)
    {

        float d1 = orient(r, static_cast<VecT>(tri.verts[0]), static_cast<VecT>(tri.verts[1]));
        float d2 = orient(r, static_cast<VecT>(tri.verts[1]), static_cast<VecT>(tri.verts[2]));
        float d3 = orient(r, static_cast<VecT>(tri.verts[2]), static_cast<VecT>(tri.verts[0]));

        return (d1 <= 0) && (d2 <= 0) && (d3 <= 0);
    }

} // namespace cdt

#endif // BOIDS_TRIANGULATION_H
