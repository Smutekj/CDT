#include "../include/Triangulation.h"

#include <cassert>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <stack>

#include "../include/Utils/Vector2.h"

namespace cdt
{

    // using namespace utils;

    template <class Vertex>
    Triangulation<Vertex>::Triangulation(Vertex box_size)
        : m_boundary(box_size)
    {
        createBoundary(box_size);
    }

    //! \brief Clears all vertices triangles and edges
    template <class Vertex>
    void Triangulation<Vertex>::reset()
    {
        m_vertices.clear();
        m_triangles.clear();
        m_tri_ind2vert_inds.clear();
        m_fixed_edges.clear();
        m_last_found = 0;

        std::fill(m_cell2tri_ind.begin(), m_cell2tri_ind.end(), -1);
        createBoundary(m_boundary);
    }

    //! \brief searches for triangle containing query_point
    //! \param query_point point whose containing triangle we are looking for
    //! \param start_from_last_found whether we start looking from previously found triangle... uses search grid if false
    //!\returns index of a triangle containing query_point or -1 if no such triangle is found
    template <class Vertex>
    TriInd Triangulation<Vertex>::findTriangleFromVertex(Vertex query_point, bool from_last_found)
    {
        TriInd tri_ind = m_last_found;
        if (!from_last_found)
        {
            auto cell_ind = m_grid->coordToCell(asFloat(query_point));
            tri_ind = m_cell2tri_ind.at(cell_ind);
            if (tri_ind == -1)
            { //! if there is no triangle yet in a cell we default to linear search
                for (TriInd i = 0; i < m_triangles.size(); ++i)
                {
                    if (isInTriangle(query_point, m_triangles[i]))
                    {
                        m_last_found = i;
                        return i;
                    }
                }
            }
        }

        Vertex v_current;

        if (isInTriangle(query_point, m_triangles[tri_ind]))
        {
            return tri_ind;
        }
        { //!  we look at triangle edges and find in which direction we need to go
            const auto &tri = m_triangles[tri_ind];

            const auto v0 = asFloat(tri.verts[0]);
            const auto v1 = asFloat(tri.verts[1]);
            const auto v2 = asFloat(tri.verts[2]);
            const auto r_start = (v0 + v1 + v2) / 3.f;

            if (segmentsIntersectOrTouch(r_start, asFloat(query_point), v0, v1))
            {
                tri_ind = tri.neighbours[0];
                v_current = tri.verts[0];
            }
            else if (segmentsIntersectOrTouch(r_start, asFloat(query_point), v1, v2))
            {
                tri_ind = tri.neighbours[1];
                v_current = tri.verts[1];
            }
            else if (segmentsIntersectOrTouch(r_start, asFloat(query_point), v2, v0))
            {
                tri_ind = tri.neighbours[2];
                v_current = tri.verts[2];
            }
            else
            {
                tri_ind = -1;
            } // throw std::runtime_error("we should never get here!");}
        }
        if (tri_ind == -1)
        { //! if we cannot find way (no idea why yet) we do brute force
            for (TriInd i = 0; i < m_triangles.size(); ++i)
            {
                if (isInTriangle(query_point, m_triangles[i]))
                {
                    m_last_found = i;
                    return i;
                }
            }
            return 0;
        }
        //! walk in the found diraction to triangle containing end_ind;
        while (!isInTriangle(query_point, m_triangles[tri_ind]))
        {
            const auto &tri = m_triangles[tri_ind];

            const auto index_in_tri = indexOf(v_current, tri);
            assert(index_in_tri != -1);

            const auto v1 = asFloat(tri.verts[next(index_in_tri)]);
            const auto v2 = asFloat(tri.verts[prev(index_in_tri)]);

            if (segmentsIntersectOrTouch(asFloat(v_current), asFloat(query_point), v1, v2))
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
            else
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
        }

        m_last_found = tri_ind;
        return tri_ind;
    }

    //! \brief searches for triangle containing query_point
    //! \param query_point point whose containing triangle we are looking for
    //! \param start_from_last_found whether we start looking from previously found triangle... uses search grid if false
    //!\returns index of a triangle containing query_point or -1 if no such triangle is found
    template <class Vertex>
    TriInd Triangulation<Vertex>::findTriangle(cdt::Vector2f query_point, bool from_last_found)
    {

        if (!isWithinBoundary(query_point))
        {
            return -1;
        }

        TriInd tri_ind = m_last_found;
        if (!from_last_found)
        {
            auto cell_ind = m_grid->coordToCell(query_point);
            tri_ind = m_cell2tri_ind.at(cell_ind);
            if (tri_ind == -1)
            { //! if there is no triangle yet in a cell we default to linear search
                for (TriInd i = 0; i < m_triangles.size(); ++i)
                {
                    if (isInTriangle(query_point, m_triangles[i]))
                    {
                        m_last_found = i;
                        return i;
                    }
                }
            }
        }

        Vertex v_current;

        if (isInTriangle(query_point, m_triangles[tri_ind]))
        {
            return tri_ind;
        }
        { //!  we look at triangle edges and find in which direction we need to go
            const auto &tri = m_triangles[tri_ind];

            const auto v0 = asFloat(tri.verts[0]);
            const auto v1 = asFloat(tri.verts[1]);
            const auto v2 = asFloat(tri.verts[2]);
            const cdt::Vector2f r_start = (v0 + v1 + v2) / 3.f;

            if (segmentsIntersectOrTouch(r_start, query_point, v0, v1))
            {
                tri_ind = tri.neighbours[0];
                v_current = tri.verts[0];
            }
            else if (segmentsIntersectOrTouch(r_start, query_point, v1, v2))
            {
                tri_ind = tri.neighbours[1];
                v_current = tri.verts[1];
            }
            else if (segmentsIntersectOrTouch(r_start, query_point, v2, v0))
            {
                tri_ind = tri.neighbours[2];
                v_current = tri.verts[2];
            }
            else
            {
                tri_ind = -1;
            } // throw std::runtime_error("we should never get here!");}
        }
        if (tri_ind == -1)
        { //! if we cannot find way (no idea why yet) we do brute force
            for (TriInd i = 0; i < m_triangles.size(); ++i)
            {
                if (isInTriangle(query_point, m_triangles[i]))
                {
                    m_last_found = i;
                    return i;
                }
            }
            return -1;
        }
        //! walk in the found diraction to triangle containing query;
        while (!isInTriangle(query_point, m_triangles.at(tri_ind)))
        {
            const auto &tri = m_triangles[tri_ind];

            const auto index_in_tri = indexOf(v_current, tri);
            assert(index_in_tri != -1);

            const auto &v1 = asFloat(tri.verts[next(index_in_tri)]);
            const auto &v2 = asFloat(tri.verts[prev(index_in_tri)]);

            if (segmentsIntersectOrTouch(asFloat(v_current), query_point, v1, v2))
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
            else
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
        }

        m_last_found = tri_ind; //! cache the result
        return tri_ind;
    }

    //! \brief creates supertriangle which contains specified boundary then
    //! \param boundary dimensions of a boundary contained in supertriangle
    template <class Vertex>
    void Triangulation<Vertex>::createSuperTriangle(cdt::Vector2i box_size)
    {
        m_boundary = box_size;

        m_grid = std::make_unique<Grid>(cdt::Vector2i{20, 20}, box_size);
        m_cell2tri_ind.resize(m_grid->getNCells(), -1);

        Triangle<Vertex> super_triangle;
        m_tri_ind2vert_inds.push_back({0, 1, 2});

        Vertex super_tri0 = {m_boundary.x / 2, m_boundary.y * 3};
        Vertex super_tri1 = {3 * m_boundary.x, -m_boundary.y / 2};
        Vertex super_tri2 = {-3 * m_boundary.x, -m_boundary.y / 2};

        super_triangle.verts[0] = super_tri0;
        super_triangle.verts[1] = super_tri1;
        super_triangle.verts[2] = super_tri2;
        m_triangles.push_back(super_triangle);

        m_vertices.push_back(super_tri0);
        m_vertices.push_back(super_tri1);
        m_vertices.push_back(super_tri2);
    }

    //! \brief creates supertriangle which contains specified boundary then
    //!        inserts 4 vertices corresponding to the boundary (upper left point is [0,0])
    //! \param boundary dimensions of a boundary contained in supertriangle
    template <class Vertex>
    void Triangulation<Vertex>::createBoundaryAndSuperTriangle(cdt::Vector2i box_size)
    {

        m_grid = std::make_unique<Grid>(cdt::Vector2i{20, 20}, box_size);
        m_cell2tri_ind.resize(m_grid->getNCells(), -1);

        Triangle<Vertex> super_triangle;
        m_tri_ind2vert_inds.push_back({0, 1, 2});

        Vertex super_tri0 = {m_boundary.x / 2, m_boundary.y * 3};
        Vertex super_tri1 = {3 * m_boundary.x, -m_boundary.y / 2};
        Vertex super_tri2 = {-3 * m_boundary.x, -m_boundary.y / 2};

        super_triangle.verts[0] = super_tri0;
        super_triangle.verts[1] = super_tri1;
        super_triangle.verts[2] = super_tri2;
        m_triangles.push_back(super_triangle);

        m_vertices.push_back(super_tri0);
        m_vertices.push_back(super_tri1);
        m_vertices.push_back(super_tri2);

        Vertex v1 = {0, 0};
        Vertex v2 = Vertex{m_boundary.x, 0};
        Vertex v3 = m_boundary;
        Vertex v4 = Vertex{0, m_boundary.y};
        insertVertex(v1);
        assert(triangulationIsConsistent());
        insertVertex(v2);
        assert(triangulationIsConsistent());
        insertVertex(v3);
        assert(triangulationIsConsistent());
        insertVertex(v4);
        assert(triangulationIsConsistent());

        EdgeVInd e1 = {3, 4};
        EdgeVInd e2 = {4, 5};
        EdgeVInd e3 = {5, 6};
        EdgeVInd e4 = {6, 3};

        insertConstraint(e1);
        insertConstraint(e2);
        insertConstraint(e3);
        insertConstraint(e4);

        assert(triangulationIsConsistent());
    }

    //! \brief creates supertriangle which contains specified boundary then
    //!        inserts 4 vertices corresponding to the boundary (upper left point is [0,0])
    //! \param boundary dimensions of a boundary contained in supertriangle
    template <class Vertex>
    void Triangulation<Vertex>::createBoundary(cdt::Vector2i box_size)
    {

        m_boundary = box_size;

        m_grid = std::make_unique<Grid>(cdt::Vector2i{20, 20}, box_size);
        m_cell2tri_ind.resize(m_grid->getNCells(), -1);

        Triangle<Vertex> tri_up;
        Triangle<Vertex> tri_down;
        m_tri_ind2vert_inds.push_back({0, 2, 1});
        m_tri_ind2vert_inds.push_back({0, 3, 2});

        Vertex v0 = {0, 0};
        Vertex v1 = Vertex{m_boundary.x, 0};
        Vertex v2 = m_boundary;
        Vertex v3 = Vertex{0, m_boundary.y};
        tri_up.verts[0] = v0;
        tri_up.verts[1] = v2;
        tri_up.verts[2] = v1;
        tri_up.neighbours[0] = 1;
        tri_up.neighbours[1] = -1u;
        tri_up.neighbours[2] = -1u;
        tri_up.is_constrained[0] = false;
        tri_up.is_constrained[1] = true;
        tri_up.is_constrained[2] = true;

        //! initialize lower triangle
        tri_down.verts[0] = v0;
        tri_down.verts[1] = v3;
        tri_down.verts[2] = v2;
        tri_down.neighbours[0] = -1u;
        tri_down.neighbours[1] = -1u;
        tri_down.neighbours[2] = 0;
        tri_down.is_constrained[0] = true;
        tri_down.is_constrained[1] = true;
        tri_down.is_constrained[2] = false;

        m_triangles.push_back(tri_up);
        m_triangles.push_back(tri_down);

        m_vertices.push_back(v0);
        m_vertices.push_back(v1);
        m_vertices.push_back(v2);
        m_vertices.push_back(v3);

        EdgeVInd e1 = {0, 1};
        EdgeVInd e2 = {1, 2};
        EdgeVInd e3 = {2, 3};
        EdgeVInd e4 = {3, 0};

        m_fixed_edges.insert({v0, v1});
        m_fixed_edges.insert({v1, v2});
        m_fixed_edges.insert({v2, v3});
        m_fixed_edges.insert({v3, v0});

        assert(triangulationIsConsistent());
    }

    //! \param np index of the neighbouring triangle
    //! \param tri
    //! \returns index in triangle of the vertex in tri opposite of triangle \p np
    template <class Vertex>
    int Triangulation<Vertex>::oppositeIndex(const TriInd np, const Triangle<Vertex> &tri)
    {
        if (np == tri.neighbours[0])
        {
            return 2;
        }
        else if (np == tri.neighbours[1])
        {
            return 0;
        }
        else if (np == tri.neighbours[2])
        {
            return 1;
        }
        else
        {
            throw std::runtime_error("triangles not neighbours!");
        }
    }

    //! \param tri  trian
    //! \param e    edge containing vertex coordinates it connects
    //! \returns index of triangle opposite of \p tri accross \p edge
    template <class Vertex>
    TriInd Triangulation<Vertex>::triangleOppositeOfEdge(const Triangle<Vertex> &tri, const EdgeI<Vertex> &e) const
    {
        const auto i2 = indexOf(e.from, tri);
        const auto i1 = indexOf(e.to(), tri);
        if (i1 == -1 or i2 == -1)
        {
            return -1;
        }
        assert(i1 != -1 && i2 != -1);
        return tri.neighbours[next(oppositeOfEdge(tri, e))];
    }

    //! \param e1 edge containg indices of vertices forming a first edge
    //! \param e2 edge containg indices of vertices forming a second edge
    //! \returns true if lines representing given edges intersect
    template <class Vertex>
    bool Triangulation<Vertex>::edgesIntersect(const EdgeVInd e1, const EdgeVInd e2) const noexcept
    {
        return segmentsIntersect(m_vertices[e1.from], m_vertices[e1.to], m_vertices[e2.from], m_vertices[e2.to]);
    }

    //! \param e1 edge containg vertices forming a first edge
    //! \param e2 edge containg vertices forming a second edge
    //! \returns true if lines representing given edges intersect
    template <class Vertex>
    bool Triangulation<Vertex>::edgesIntersect(const EdgeI<Vertex> e1, const EdgeI<Vertex> e2) const noexcept
    {
        return segmentsIntersect(e1.from, e1.to(), e2.from, e2.to());
    }

    //! \brief updates search grid used to find triangles
    //! \brief should be called whenever triagulation changes
    template <class Vertex>
    void Triangulation<Vertex>::updateCellGrid()
    {
        const auto dx = m_grid->m_cell_size.x;
        const auto dy = m_grid->m_cell_size.y;

        const auto n_cells_x = m_grid->m_cell_count.x;
        const auto n_cells_y = m_grid->m_cell_count.y;
        for (int j = 0; j < n_cells_y - 1; j++)
        { //! we walk zig-zag so that each next cell grid is close to the last one which helps findTriangle
            for (int i = 0; i < n_cells_x; i++)
            {
                const cdt::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
                const auto cell_ind = j * n_cells_x + i;

                bool from_previous_one = true;
                m_cell2tri_ind.at(cell_ind) = findTriangle(r_center, from_previous_one);
            }

            j++;
            for (int i = n_cells_x - 1; i >= 0; i--)
            {
                const cdt::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
                const auto cell_ind = j * n_cells_x + i;

                bool from_previous_one = true;
                m_cell2tri_ind.at(cell_ind) = findTriangle(r_center, from_previous_one);
            }
        }
        if (n_cells_y % 2 == 1)
        {
            int j = n_cells_y - 1;
            for (int i = n_cells_x - 1; i >= 0; i--)
            {
                const cdt::Vector2f r_center = {i * dx + dx / 2.f, j * dy + dy / 2.f};
                const auto cell_ind = j * n_cells_x + i;

                bool from_previous_one = true;
                m_cell2tri_ind.at(cell_ind) = findTriangle(r_center, from_previous_one);
            }
        }
    }

    //! \brief inserts vertex given we know that it lies directly on the given edge
    //! \param v_ind index of inserted vertex
    //! \param tri_ind_a index of counterclockwise triangle
    //! \param tri_ind_b index of a clockwise triangle
    //! \param edge
    template <class Vertex>
    void Triangulation<Vertex>::insertVertexOnEdge(const Vertex &new_vertex, TriInd tri_ind_a, TriInd tri_ind_b, const EdgeI<Vertex> &e)
    {
        const auto new_vertex_ind = m_vertices.size() - 1;

        const auto tri_ind_a_new = m_triangles.size();
        const auto tri_ind_b_new = m_triangles.size() + 1;

        auto tri_a = m_triangles.at(tri_ind_a);
        auto tri_b = m_triangles.at(tri_ind_b);

        const auto ind_in_tri_a = oppositeOfEdge(tri_a, e);
        const auto ind_in_tri_b = oppositeOfEdge(tri_b, e);

        Triangle<Vertex> tri_a_new = tri_a;
        Triangle<Vertex> tri_b_new = tri_b;

        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind_a]);
        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind_b]);

        for (int i = 0; i < 3; ++i)
        {
            tri_a_new.is_constrained[i] = false;
        }
        for (int i = 0; i < 3; ++i)
        {
            tri_b_new.is_constrained[i] = false;
        }

        //! initialize new triangles (consult an image to understand)
        m_tri_ind2vert_inds[tri_ind_a_new][next(ind_in_tri_a)] = new_vertex_ind;
        tri_a_new.verts[next(ind_in_tri_a)] = new_vertex;
        tri_a_new.neighbours[ind_in_tri_a] = tri_ind_a;
        tri_a_new.neighbours[next(ind_in_tri_a)] = tri_ind_b;
        tri_a_new.is_constrained[next(ind_in_tri_a)] = true;
        //! we inherit constraint from tri_a
        tri_a_new.is_constrained[prev(ind_in_tri_a)] = tri_a.is_constrained[prev(ind_in_tri_a)];

        //! update old ones
        m_tri_ind2vert_inds[tri_ind_a][prev(ind_in_tri_a)] = new_vertex_ind;
        tri_a.verts[prev(ind_in_tri_a)] = new_vertex;
        tri_a.neighbours[next(ind_in_tri_a)] = tri_ind_b_new;
        tri_a.neighbours[prev(ind_in_tri_a)] = tri_ind_a_new;
        tri_a.is_constrained[prev(ind_in_tri_a)] = false;

        m_tri_ind2vert_inds[tri_ind_b_new][next(ind_in_tri_b)] = new_vertex_ind;
        tri_b_new.verts[next(ind_in_tri_b)] = new_vertex;
        tri_b_new.neighbours[ind_in_tri_b] = tri_ind_b;
        tri_b_new.neighbours[next(ind_in_tri_b)] = tri_ind_a;
        tri_b_new.is_constrained[next(ind_in_tri_b)] = true;
        tri_b_new.is_constrained[prev(ind_in_tri_b)] = tri_b.is_constrained[prev(ind_in_tri_b)];

        m_tri_ind2vert_inds[tri_ind_b][prev(ind_in_tri_b)] = new_vertex_ind;
        tri_b.verts[prev(ind_in_tri_b)] = new_vertex;
        tri_b.neighbours[next(ind_in_tri_b)] = tri_ind_a_new;
        tri_b.neighbours[prev(ind_in_tri_b)] = tri_ind_b_new;
        tri_b.is_constrained[prev(ind_in_tri_b)] = false;

        //! we tell old triangles that they have a new neighbour;
        updateIndsOfNeighbour(tri_a_new.neighbours[prev(ind_in_tri_a)], tri_ind_a, tri_ind_a_new);
        updateIndsOfNeighbour(tri_b_new.neighbours[prev(ind_in_tri_b)], tri_ind_b, tri_ind_b_new);

        //! insert triangles and update
        m_triangles.push_back(tri_a_new);
        m_triangles.push_back(tri_b_new);
        m_triangles[tri_ind_a] = tri_a;
        m_triangles[tri_ind_b] = tri_b;
        //! we inserted into an existing edge, so we split it to get two new ones and delete the old one
        m_fixed_edges.erase(e);
        m_fixed_edges.insert({e.from, new_vertex});
        m_fixed_edges.insert({new_vertex, e.to()});

        //! gather triangles that changed to fix their Delaunay property
        std::stack<std::pair<TriInd, TriInd>> triangles_to_fix;
        triangles_to_fix.push({tri_ind_a_new, tri_a_new.neighbours[prev(ind_in_tri_a)]});
        triangles_to_fix.push({tri_ind_a, tri_a.neighbours[ind_in_tri_a]});
        triangles_to_fix.push({tri_ind_b_new, tri_b_new.neighbours[prev(ind_in_tri_b)]});
        triangles_to_fix.push({tri_ind_b, tri_b.neighbours[ind_in_tri_b]});

        fixDelaunayProperty(new_vertex, triangles_to_fix);
    }

    //! \param new_vertex
    //! \param tri_ind triangle index of an existing triangle containing new_vertex
    //! \returns index of the overlapping vertex
    //! \returns -1 in case there is no existing overlapping vertex
    template <class Vertex>
    VertInd Triangulation<Vertex>::findOverlappingVertex(const Vertex &new_vertex, const TriInd tri_ind) const
    {
        assert(tri_ind != -1);
        const auto old_triangle = m_triangles[tri_ind];
        const auto v0 = old_triangle.verts[0];
        const auto v1 = old_triangle.verts[1];
        const auto v2 = old_triangle.verts[2];
        if (v0 == new_vertex)
        {
            return m_tri_ind2vert_inds[tri_ind][0];
        }
        if (v1 == new_vertex)
        {
            return m_tri_ind2vert_inds[tri_ind][1];
        }
        if (v2 == new_vertex)
        {
            return m_tri_ind2vert_inds[tri_ind][2];
        }
        return -1;
    }

    //! \brief checks whether \p new_vertex lies on some existing edge
    //! \param new_vertex
    //! \param tri_ind triangle index of an existing triangle containing new_vertex
    //! \returns overlapping edge
    //! \returns {-1, -1} in case there is no overlapping edge
    template <class Vertex>
    EdgeVInd Triangulation<Vertex>::findOverlappingEdge(const Vertex &new_vertex, const TriInd tri_ind) const
    {
        const auto &old_triangle = m_triangles[tri_ind];
        const auto v0 = old_triangle.verts[0];
        const auto v1 = old_triangle.verts[1];
        const auto v2 = old_triangle.verts[2];

        const auto &vert_inds = m_tri_ind2vert_inds[tri_ind];

        EdgeI edge0 = {old_triangle.verts[0], old_triangle.verts[1]};
        EdgeI edge1 = {old_triangle.verts[1], old_triangle.verts[2]};
        EdgeI edge2 = {old_triangle.verts[2], old_triangle.verts[0]};
        if (old_triangle.is_constrained[0] && liesBetween(new_vertex, v0, v1))
        {
            return {vert_inds[0], vert_inds[1]};
        }
        else if (old_triangle.is_constrained[1] && liesBetween(new_vertex, v1, v2))
        {
            return {vert_inds[1], vert_inds[2]};
        }
        else if (old_triangle.is_constrained[2] && liesBetween(new_vertex, v2, v0))
        {
            return {vert_inds[2], vert_inds[0]};
        }
        return EdgeVInd();
    }

    //! \brief inserts \p new_vertex into triangulation
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    template <class Vertex>
    void Triangulation<Vertex>::insertVertex(const Vertex &new_vertex, bool search_from_last_one)
    {

        auto data = insertVertexAndGetData(new_vertex, search_from_last_one);
        assert(allTrianglesValid());
    }

    //! \brief inserts \p new_vertex into triangulation, the inserted vertex can either:
    //! \brief already exist, or it may lie on an existing edge or it lies in free space
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    //! \returns data relating to the actual type of insertion performed
    template <class Vertex>
    VertexInsertionData Triangulation<Vertex>::insertVertexAndGetData(int vx, int vy, bool search_from_last_one)
    {
        return insertVertexAndGetData({vx, vy}, search_from_last_one);
    }

    //! \brief inserts all \p verts into the triangulation
    //!
    template <class Vertex>
    void Triangulation<Vertex>::insertVertices(const std::vector<Vertex> &verts)
    {
        int vert_ind = m_vertices.size();
        std::vector<std::vector<Vertex>> m_grid2vert_inds(m_grid->getNCells());

        //! sort vertices into bins formed by grid cells
        for (auto &v : verts)
        {
            m_grid2vert_inds.at(m_grid->cellIndex(v)).push_back(v);
            vert_ind++;
        }

        auto cells_x = m_grid->m_cell_count.x;
        for (int iy = 0; iy < m_grid->m_cell_count.y; iy++)
        {
            bool odd_line = iy % 2 == 1;
            for (int ix = 0; ix < m_grid->m_cell_count.x; ix++)
            {

                int ix_walk = odd_line * (cells_x - 1) + (1 - 2 * odd_line) * ix;
                auto cell_ind = m_grid->cellIndex(ix_walk, iy);
                for (auto &v : m_grid2vert_inds.at(cell_ind))
                {
                    auto data = insertVertexAndGetData(v, true);
                }
            }
        }
    }

    //! \brief inserts \p new_vertex into triangulation, the inserted vertex can either:
    //! \brief already exist, or it may lie on an existing edge or it lies in free space
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    //! \returns data relating to the actual type of insertion performed
    template <class Vertex>
    VertexInsertionData Triangulation<Vertex>::insertVertexAndGetData(const Vertex &new_vertex, bool search_from_last_one)
    {
        VertexInsertionData data;

        if (!isWithinBoundary(new_vertex))
        {
            return data;
        }

        auto tri_ind = findTriangleFromVertex(new_vertex, search_from_last_one);
        const auto old_triangle = m_triangles[tri_ind];

        data.overlapping_vertex = findOverlappingVertex(new_vertex, tri_ind);
        if (data.overlapping_vertex != -1)
        {
            //! we inserted on an existing vertex
            return data;
        }

        const auto new_vertex_ind = m_vertices.size();
        m_vertices.push_back(new_vertex);

        const auto overlapping_edge = findOverlappingEdge(new_vertex, tri_ind);
        if (overlapping_edge.from != -1)
        {
            //! we inserted on an existing constrained edge
            const EdgeI edge_i = {m_vertices[overlapping_edge.from], m_vertices[overlapping_edge.to]};
            insertVertexOnEdge(new_vertex, tri_ind, triangleOppositeOfEdge(old_triangle, edge_i), edge_i);
            data.overlapping_edge = overlapping_edge;
            return data;
        }

        insertVertexIntoSpace(new_vertex, tri_ind, new_vertex_ind);
        return data;
    }

    //! \brief inserts \p new_vertex into triangulation knowing it lies in free space (not on edge or vertex)
    //! \param new_vertex
    //! \param search_from_last_one should be true if last added vertex is not far from \p new_vertex
    //! \returns data relating to the actual type of insertion performed
    template <class Vertex>
    void Triangulation<Vertex>::insertVertexIntoSpace(const Vertex &new_vertex, TriInd tri_ind, VertInd new_vertex_ind)
    {

        const auto old_triangle = m_triangles[tri_ind];

        const auto first_new_triangle_ind = tri_ind;
        const auto second_new_triangle_ind = m_triangles.size();
        const auto third_new_triangle_ind = m_triangles.size() + 1;

        Triangle t1_new = old_triangle;
        Triangle t2_new = old_triangle;
        Triangle t3_new = old_triangle;

        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind]);
        m_tri_ind2vert_inds.push_back(m_tri_ind2vert_inds[tri_ind]);

        for (int i = 0; i < 3; ++i)
        {
            t1_new.is_constrained[i] = false;
        }
        for (int i = 0; i < 3; ++i)
        {
            t2_new.is_constrained[i] = false;
        }
        for (int i = 0; i < 3; ++i)
        {
            t3_new.is_constrained[i] = false;
        }
        //    m_tri_ind2vert_inds[first_new_triangle_ind] = new_vertex_ind;
        m_tri_ind2vert_inds[first_new_triangle_ind][2] = new_vertex_ind;
        t1_new.verts[2] = new_vertex;
        t1_new.neighbours[2] = third_new_triangle_ind;
        t1_new.neighbours[1] = second_new_triangle_ind;
        if (old_triangle.is_constrained[0])
        {
            t1_new.is_constrained[0] = true;
        }

        m_triangles[tri_ind] = t1_new;

        //    t2_new.vertinds[0] =  new_vertex_ind;
        m_tri_ind2vert_inds[second_new_triangle_ind][0] = new_vertex_ind;
        t2_new.verts[0] = new_vertex;
        t2_new.neighbours[0] = first_new_triangle_ind;
        t2_new.neighbours[2] = third_new_triangle_ind;
        if (old_triangle.is_constrained[1])
        {
            t2_new.is_constrained[1] = true;
        }

        //    t3_new.vertinds[1] = new_vertex_ind;
        m_tri_ind2vert_inds[third_new_triangle_ind][1] = new_vertex_ind;
        t3_new.verts[1] = new_vertex;
        t3_new.neighbours[1] = second_new_triangle_ind;
        t3_new.neighbours[0] = first_new_triangle_ind;
        if (old_triangle.is_constrained[2])
        {
            t3_new.is_constrained[2] = true;
        }

        //! we tell old triangles that they have a new neighbour;
        //! since we create only two new triangles we need to do just two updates
        updateIndsOfNeighbour(old_triangle.neighbours[1], tri_ind, second_new_triangle_ind);
        updateIndsOfNeighbour(old_triangle.neighbours[2], tri_ind, third_new_triangle_ind);

        //! fix delaunay property
        std::stack<std::pair<TriInd, TriInd>> triangles_to_fix;
        triangles_to_fix.push({first_new_triangle_ind, old_triangle.neighbours[0]});
        triangles_to_fix.push({second_new_triangle_ind, old_triangle.neighbours[1]});
        triangles_to_fix.push({third_new_triangle_ind, old_triangle.neighbours[2]});

        m_triangles.push_back(t2_new);
        m_triangles.push_back(t3_new);

        fixDelaunayProperty(new_vertex, triangles_to_fix);
    }

    //! \returns true if quadrilateral formed by the giver four vertices is convex
    template <class Vertex>
    bool Triangulation<Vertex>::isConvex(const Vertex v1, const Vertex v2, const Vertex v3,
                                         const Vertex v4) const
    { //! v1-v3 and v2-v4 are diagonals
        return segmentsIntersect((cdt::Vector2i)v1, (cdt::Vector2i)v3, (cdt::Vector2i)v2, (cdt::Vector2i)v4);
    }

    //! \param edge represented by vertex coordinates
    //! \param tri
    //! \returns index in triangle of the vertex in \p tri which is opposite of the \p edge
    template <class Vertex>
    int Triangulation<Vertex>::oppositeOfEdge(const Triangle<Vertex> &tri, const EdgeI<Vertex> &e) const
    {
        const auto i1 = indexOf(e.from, tri);
        const auto i2 = indexOf(e.to(), tri);
        return 3 - (i1 + i2); //! i1 + i2 has values 1,2,3.. corresponding output should be 2,1,0
    }

    //! \brief checks if neighbours of each triangles are consistent and
    //! \brief if m_tri_ind2vert_inds points to right place in m_vertices array
    template <class Vertex>
    bool Triangulation<Vertex>::triangulationIsConsistent() const
    {
        for (int tri_ind = 0; tri_ind < m_triangles.size(); ++tri_ind)
        {
            const auto &tri = m_triangles[tri_ind];
            for (int k = 0; k < 3; ++k)
            {
                const auto neighbour_tri_ind = tri.neighbours[k];
                if (neighbour_tri_ind != -1)
                {
                    const auto &neighbour_tri = m_triangles[neighbour_tri_ind];
                    const auto ind_in_neirhbour_tri = 0;
                    const auto found_at = indInTriOf(neighbour_tri, tri_ind);
                    // std::find(neighbour_tri.neighbours.begin(), neighbour_tri.neighbours.end(), tri_ind) -
                    // neighbour_tri.neighbours.begin();
                    if (found_at < 0 or found_at > 2)
                    {
                        return false;
                    }
                    if (tri.is_constrained[k] xor neighbour_tri.is_constrained[found_at])
                    {
                        return false;
                    }
                }
                const auto v = tri.verts[k];
                if (v != m_vertices[m_tri_ind2vert_inds[tri_ind][k]])
                {
                    return false;
                }
            }
        }
        return true;
    }

    //! \brief forces triangulation to have a constrained edge connecting \p e.from and \p e.to
    //! \param e edge representing the constraint
    template <class Vertex>
    void Triangulation<Vertex>::insertConstraint(const EdgeVInd e)
    {

        auto vi_ind = e.from;
        auto vj_ind = e.to;
        auto vi = m_vertices[vi_ind];
        auto vj = m_vertices[vj_ind];
        if (m_fixed_edges.count({vi, vj}) > 0 || e.from == e.to) //! constrained edge alread exists
        {
            return;
        }
        m_fixed_edges.insert({vi, vj});

        std::deque<EdgeI<Vertex>> intersected_edges;
        std::deque<TriInd> intersected_tri_inds;
        findIntersectingEdges(e, intersected_edges, intersected_tri_inds);

        auto overlaps = findOverlappingEdges(vi, vj);
        if (!overlaps.empty())
        {
            for (const auto &overlap : overlaps)
            {
                insertConstraint(overlap);
            }
            insertConstraint({e.from, overlaps[0].from});
            for (int i = 1; i < overlaps.size(); ++i)
            {
                insertConstraint({overlaps.at(i - 1).to, overlaps.at(i).from});
            }
            insertConstraint({e.to, overlaps.back().to});
            return;
        }

        std::vector<EdgeI<Vertex>> newly_created_edges;
        std::vector<std::pair<TriInd, TriInd>> newly_created_edge_tris;

        EdgeI e_inserted = {vi, vj};

        //! remove intersecting edges
        while (!intersected_edges.empty())
        {
            auto tri_ind = intersected_tri_inds.front();
            auto &tri = m_triangles[tri_ind];
            intersected_tri_inds.pop_front();
            auto e_next = intersected_edges.front();
            intersected_edges.pop_front();

            auto next_tri_ind = triangleOppositeOfEdge(tri, e_next);
            auto &next_tri = m_triangles[next_tri_ind];

            auto v_current_ind = m_tri_ind2vert_inds[tri_ind][oppositeOfEdge(tri, e_next)];
            auto v_current = tri.verts[oppositeOfEdge(tri, e_next)];
            auto v_opposite_ind_in_tri = oppositeIndex(tri_ind, next_tri);
            auto v_opposite_current = next_tri.verts[v_opposite_ind_in_tri];

            //! we can swap edges only in convex quadrilaterals otherwise bad shapes get created
            if (isConvex(v_current, e_next.from, v_opposite_current, e_next.to()))
            {
                if (isCounterClockwise(v_opposite_current, vi, vj))
                {
                    swapConnectingEdgeCounterClockwise(next_tri_ind, tri_ind);
                    assert(triangulationIsConsistent());
                }
                else
                {
                    swapConnectingEdgeClockwise(tri_ind, next_tri_ind);
                    assert(triangulationIsConsistent());
                }
                assert(allTrianglesValid());

                e_next = {v_current, v_opposite_current};

                if (edgesIntersect(e_next, e_inserted))
                {
                    intersected_edges.push_back(e_next);
                    intersected_tri_inds.push_back(tri_ind);
                    auto ind_in_tri = oppositeOfEdge(m_triangles.at(tri_ind), e_next);
                }
                else
                {
                    newly_created_edges.push_back(e_next);
                    newly_created_edge_tris.push_back({tri_ind, next_tri_ind});
                }
            }
            else
            {
                intersected_edges.push_back(e_next);
                intersected_tri_inds.push_back(tri_ind);
            }
        }

        //! Fix Delaunay triangulation (Steps 4.1 - 4.3)
        bool some_swap_happened = true;
        while (some_swap_happened)
        {
            some_swap_happened = false;

            for (int i = 0; i < newly_created_edges.size(); ++i)
            {
                const auto &e_new = newly_created_edges[i];
                const auto tri_ind_a = newly_created_edge_tris[i].first;
                auto &tri_a = m_triangles[tri_ind_a];
                auto tri_ind_b = triangleOppositeOfEdge(tri_a, e_new);
                if (tri_ind_b == -1)
                {
                    tri_ind_b = newly_created_edge_tris[i].second;
                }
                auto &tri_b = m_triangles[tri_ind_b];

                const auto opposite_ind_in_tri_a = oppositeOfEdge(tri_a, e_new);
                const auto opposite_ind_in_tri_b = oppositeOfEdge(tri_b, e_new);

                vi = tri_a.verts[opposite_ind_in_tri_a];
                vj = tri_b.verts[opposite_ind_in_tri_b];
                vi_ind = m_tri_ind2vert_inds[tri_ind_a][opposite_ind_in_tri_a];
                auto vi_ind_next = m_tri_ind2vert_inds[tri_ind_a][next(opposite_ind_in_tri_a)];
                auto vi_ind_prev = m_tri_ind2vert_inds[tri_ind_a][prev(opposite_ind_in_tri_a)];

                if (e_new == e_inserted)
                {
                    tri_a.is_constrained[next(opposite_ind_in_tri_a)] = true;
                    tri_b.is_constrained[next(opposite_ind_in_tri_b)] = true;
                    continue;
                }

                const auto v1 = m_vertices.at(vi_ind_next);
                const auto v2 = m_vertices.at(vi_ind_prev);

                bool edge_needs_swap = needSwap(vi, v1, v2, vj);
                bool is_convex = true; //! Convexity check should be automatically taken care of by needSwap but it doesn't
                                       //! and I don't know why yet :(
                if (!isConvex(m_vertices[vi_ind], v1, v2, m_vertices[vj_ind]))
                {
                    is_convex = false;
                }
                if (edge_needs_swap && is_convex)
                {

                    swapConnectingEdgeClockwise(tri_ind_a, tri_ind_b);
                    assert(allTrianglesValid());

                    some_swap_happened = true;
                    newly_created_edges[i] = {vi, vj};
                }
            }
        }
    }

    //! \brief swaps edge connecting \p tri_ind_a with tri_ind_b such that they move in a clockwise manner
    //! \param tri_ind_a index of a triangle
    //! \param tri_ind_a index of another triangle
    template <class Vertex>
    void Triangulation<Vertex>::swapConnectingEdgeClockwise(const TriInd &tri_ind_a, const TriInd &tri_ind_b)
    {
        auto &tri_a = m_triangles[tri_ind_a];
        auto &tri_b = m_triangles[tri_ind_b];

        auto v_a_ind_in_tri = oppositeIndex(tri_ind_b, tri_a);
        auto v_b_ind_in_tri = oppositeIndex(tri_ind_a, tri_b);

        //! change vertices -> prev(a) becomes b and prev(b) becomes a;
        const auto &v_b = tri_b.verts[v_b_ind_in_tri];
        const auto &v_a = tri_a.verts[v_a_ind_in_tri];
        const auto &v_left = tri_a.verts[prev(v_a_ind_in_tri)];
        const auto &v_right = tri_a.verts[next(v_a_ind_in_tri)];
        tri_a.verts[next(v_a_ind_in_tri)] = v_b;
        tri_b.verts[next(v_b_ind_in_tri)] = v_a;

        //! change neighbours
        const auto na = tri_a.neighbours[(v_a_ind_in_tri)];
        const auto nb = tri_b.neighbours[(v_b_ind_in_tri)];
        const auto na_prev = tri_a.neighbours[prev(v_a_ind_in_tri)];
        const auto nb_prev = tri_b.neighbours[prev(v_b_ind_in_tri)];
        tri_a.neighbours[next(v_a_ind_in_tri)] = nb;
        tri_a.neighbours[(v_a_ind_in_tri)] = tri_ind_b;
        tri_b.neighbours[next(v_b_ind_in_tri)] = na;
        tri_b.neighbours[(v_b_ind_in_tri)] = tri_ind_a;
        //! change constraints
        tri_a.is_constrained[next(v_a_ind_in_tri)] = tri_b.is_constrained[(v_b_ind_in_tri)];
        tri_b.is_constrained[next(v_b_ind_in_tri)] = tri_a.is_constrained[(v_a_ind_in_tri)];
        tri_a.is_constrained[(v_a_ind_in_tri)] = false;
        tri_b.is_constrained[(v_b_ind_in_tri)] = false;

        m_tri_ind2vert_inds.at(tri_ind_a)[next(v_a_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_b)[v_b_ind_in_tri];
        m_tri_ind2vert_inds.at(tri_ind_b)[next(v_b_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_a)[v_a_ind_in_tri];

        //! tell neighbours that there was a swap
        updateIndsOfNeighbour(nb, tri_ind_b, tri_ind_a);
        updateIndsOfNeighbour(na, tri_ind_a, tri_ind_b);
    }

    template <class Vertex>
    void Triangulation<Vertex>::updateIndsOfNeighbour(TriInd to_update, TriInd old_neighbour, TriInd new_neighbour)
    {
        if (to_update != -1)
        {
            auto &to_update_tri = m_triangles.at(to_update);
            auto ind_in_neighbour = indInTriOf(to_update_tri, old_neighbour);
            //! update neighbour inds of the
            to_update_tri.neighbours[ind_in_neighbour] = new_neighbour;
        }
    }

    //! \brief swaps edge connecting \p tri_ind_a with tri_ind_b such that they move in a counter-clockwise manner
    //! \param tri_ind_a index of a triangle
    //! \param tri_ind_a index of another triangle
    template <class Vertex>
    void Triangulation<Vertex>::swapConnectingEdgeCounterClockwise(const TriInd &tri_ind_a, const TriInd &tri_ind_b)
    {
        auto &tri_a = m_triangles[tri_ind_a];
        auto &tri_b = m_triangles[tri_ind_b];

        auto v_a_ind_in_tri = oppositeIndex(tri_ind_b, tri_a);
        auto v_b_ind_in_tri = oppositeIndex(tri_ind_a, tri_b);

        //! change vertices -> prev(a) becomes b and prev(b) becomes a;
        const auto &v_b = tri_b.verts[v_b_ind_in_tri];
        const auto &v_a = tri_a.verts[v_a_ind_in_tri];
        const auto &v_left = tri_a.verts[prev(v_a_ind_in_tri)];
        const auto &v_right = tri_a.verts[next(v_a_ind_in_tri)];

        tri_a.verts[prev(v_a_ind_in_tri)] = v_b;
        tri_b.verts[prev(v_b_ind_in_tri)] = v_a;

        //! change neighbours
        const auto na_prev = tri_a.neighbours[prev(v_a_ind_in_tri)];
        const auto nb_prev = tri_b.neighbours[prev(v_b_ind_in_tri)];
        tri_a.neighbours[next(v_a_ind_in_tri)] = nb_prev;
        tri_a.neighbours[prev(v_a_ind_in_tri)] = tri_ind_b;
        tri_b.neighbours[next(v_b_ind_in_tri)] = na_prev;
        tri_b.neighbours[prev(v_b_ind_in_tri)] = tri_ind_a;
        //! change constraints
        tri_a.is_constrained[next(v_a_ind_in_tri)] = tri_b.is_constrained[prev(v_b_ind_in_tri)];
        tri_b.is_constrained[next(v_b_ind_in_tri)] = tri_a.is_constrained[prev(v_a_ind_in_tri)];
        tri_a.is_constrained[prev(v_a_ind_in_tri)] = false;
        tri_b.is_constrained[prev(v_b_ind_in_tri)] = false;

        m_tri_ind2vert_inds.at(tri_ind_a)[prev(v_a_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_b)[v_b_ind_in_tri];
        m_tri_ind2vert_inds.at(tri_ind_b)[prev(v_b_ind_in_tri)] = m_tri_ind2vert_inds.at(tri_ind_a)[v_a_ind_in_tri];

        //! tell neighbours that there was a swap
        updateIndsOfNeighbour(nb_prev, tri_ind_b, tri_ind_a);
        updateIndsOfNeighbour(na_prev, tri_ind_a, tri_ind_b);
    }

    //! \brief looks for existing constrained edges that overlap with new edge vi - vj
    //! \param vi   first point of the new edge
    //! \param vj   second point of the new edge
    //! \returns    a list of edges defined by their vertex indices
    template <class Vertex>
    std::vector<EdgeI<Vertex>> Triangulation<Vertex>::findOverlappingConstraints(const Vertex &vi, const Vertex &vj)
    {

        //! walk from tri_ind_start to  tri_ind_end while looking for collinear constrained edges
        const auto start_tri_ind = findTriangleFromVertex(vi, false);
        const auto start_tri = m_triangles[start_tri_ind];
        const auto end_tri_ind = findTriangleFromVertex(vj, true);
        const auto end_tri = m_triangles[end_tri_ind];

        auto tri_ind = start_tri_ind;
        auto tri = m_triangles[tri_ind];
        auto index_in_tri = indexOf(vi, tri);

        auto v_left = tri.verts[prev(index_in_tri)];
        auto v_right = tri.verts[next(index_in_tri)];

        std::vector<EdgeI<Vertex>> overlapps;

        bool prev_touched = false;

        // check if the vj is already connected to vi
        do
        {
            if (liesBetween(v_right, vi, vj))
            {
                if (tri.is_constrained[index_in_tri])
                {
                    overlapps.push_back({vi, v_right});
                }
                break;
            }
            if (segmentsIntersect(v_left, v_right, vi, vj))
            {
                tri_ind = triangleOppositeOfEdge(tri, {v_left, v_right});
                break;
            }
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);
            v_left = v_right;
            v_right = tri.verts[next(index_in_tri)];
        } while (tri_ind != start_tri_ind);

        auto v_current = v_right;
        auto prev_tri_ind = tri_ind;
        tri_ind = tri.neighbours[next(index_in_tri)];

        auto opp_vertex = v_right;
        auto prev_opp_vertex = opp_vertex;

        //! walks towards vj and looks for overlaps
        while (v_current != vj)
        {
            auto &tri = m_triangles.at(tri_ind);
            index_in_tri = indexOf(v_current, tri);
            opp_vertex = tri.verts[next(index_in_tri)];

            if (liesBetween(v_current, vi, vj) && liesBetween(opp_vertex, vi, vj) && tri.is_constrained[index_in_tri])
            {
                overlapps.push_back({v_current, opp_vertex});
            }

            //!
            if (orient(vi, vj, opp_vertex) > 0)
            {
                v_current = tri.verts[next(index_in_tri)];
                tri_ind = tri.neighbours[next(index_in_tri)];
                prev_touched = false;
            }
            else if (strictly_less(orient(vi, vj, opp_vertex), 0))
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
            else
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
        }
        return overlapps;
    }

    //! \brief looks for existing constrained edges that overlap with new edge vi - vj
    //! \param vi   first point of the new edge
    //! \param vj   second point of the new edge
    //! \returns    a list of edges defined by their vertex indices
    template <class Vertex>
    std::vector<EdgeVInd> Triangulation<Vertex>::findOverlappingEdges(const Vertex &vi, const Vertex &vj)
    {

        //! walk from tri_ind_start to  tri_ind_end while looking for collinear constrained edges
        const auto start_tri_ind = findTriangleFromVertex(vi, false);
        const auto start_tri = m_triangles[start_tri_ind];
        const auto end_tri_ind = findTriangleFromVertex(vj, true);
        const auto end_tri = m_triangles[end_tri_ind];

        auto tri_ind = start_tri_ind;
        auto tri = m_triangles[tri_ind];
        auto index_in_tri = indexOf(vi, tri);

        auto v_left = tri.verts[prev(index_in_tri)];
        auto v_right = tri.verts[next(index_in_tri)];

        std::vector<EdgeVInd> overlapps;

        bool prev_touched = false;

        // check if the vj is already connected to vi
        do
        {
            if (liesBetween(v_right, vi, vj))
            {
                auto a = indexOf(vi, tri);
                auto b = indexOf(v_right, tri);
                auto a_ind = m_tri_ind2vert_inds.at(tri_ind)[a];
                auto b_ind = m_tri_ind2vert_inds.at(tri_ind)[b];
                overlapps.push_back({a_ind, b_ind});
                break;
            }
            if (segmentsIntersect(v_left, v_right, vi, vj))
            {
                //! we found direction where vj must lie;
                tri_ind = triangleOppositeOfEdge(tri, {v_left, v_right});
                break;
            }
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);
            v_left = v_right;
            v_right = tri.verts[next(index_in_tri)];
        } while (tri_ind != start_tri_ind);

        auto v_current = v_right;
        auto prev_tri_ind = tri_ind;
        tri_ind = tri.neighbours[next(index_in_tri)];

        auto opp_vertex = v_right;
        auto prev_opp_vertex = opp_vertex;

        //! walk towards vj and gather overlapping edges
        while (v_current != vj)
        {
            auto &tri = m_triangles.at(tri_ind);
            index_in_tri = indexOf(v_current, tri);
            opp_vertex = tri.verts[next(index_in_tri)];

            if (liesBetween(v_current, vi, vj) && liesBetween(opp_vertex, vi, vj)) // && tri.is_constrained[index_in_tri])
            {                                                                      //! opposite vertex touches the inserted constraints
                auto a = indexOf(v_current, tri);
                auto b = indexOf(opp_vertex, tri);
                auto a_ind = m_tri_ind2vert_inds.at(tri_ind)[a];
                auto b_ind = m_tri_ind2vert_inds.at(tri_ind)[b];
                if (dot(vj - vi, opp_vertex - v_current) > 0)
                {
                    overlapps.push_back({a_ind, b_ind});
                }
                else
                {
                    overlapps.push_back({b_ind, a_ind});
                }
            }

            //!
            if (orient(vi, vj, opp_vertex) > 0)
            {
                v_current = tri.verts[next(index_in_tri)];
                tri_ind = tri.neighbours[next(index_in_tri)];
                prev_touched = false;
            }
            else if (strictly_less(orient(vi, vj, opp_vertex), 0))
            {
                tri_ind = tri.neighbours[index_in_tri];
            }
            else
            {
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
        }
        return overlapps;
    }

    //! \brief finds existing edges and their corresponding triangles that would intersect with edge \p e
    //! \brief writes the edges into \p intersected_edges and triangles into \p intersected_tri_inds
    //! \param e edge containing vertex indices
    //! \param intersected_edges here the intersected edges are written;
    //! \param intersected_tri_inds here the tri inds corresponding to \p intersected_edges are written
    template <class Vertex>
    void Triangulation<Vertex>::findIntersectingEdges(const EdgeVInd &e, std::deque<EdgeI<Vertex>> &intersected_edges,
                                                      std::deque<TriInd> &intersected_tri_inds)
    {
        const auto vi_ind = e.from;
        const auto vj_ind = e.to;
        if (vi_ind == vj_ind)
        {
            return;
        }
        const auto vi = m_vertices[vi_ind];
        const auto vj = m_vertices[vj_ind];

        const auto start_tri_ind = findTriangleFromVertex(vi, false);
        const auto start_tri = m_triangles[start_tri_ind];
        const auto end_tri_ind = findTriangleFromVertex(vj, true);
        const auto end_tri = m_triangles[end_tri_ind];

        EdgeI<Vertex> e_inserted(vi, vj);

        auto tri_ind = start_tri_ind;
        auto tri = m_triangles[tri_ind];
        auto index_in_tri = indexOf(vi, tri);

        auto v_left = tri.verts[prev(index_in_tri)];
        auto v_right = tri.verts[next(index_in_tri)];

        // check if the vj is already connected to vi
        do
        {
            if (v_right == vj)
            {
                m_triangles[tri_ind].is_constrained[index_in_tri] = true;
                const auto tri_ind_opposite = triangleOppositeOfEdge(tri, e_inserted);
                const auto ind_in_opposite_tri = indexOf(vj, m_triangles[tri_ind_opposite]);
                m_triangles[tri_ind_opposite].is_constrained[ind_in_opposite_tri] = true;
                return;
            }
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);
            v_right = tri.verts[next(index_in_tri)];
        } while (tri_ind != start_tri_ind);

        tri_ind = start_tri_ind;
        tri = m_triangles[tri_ind];
        index_in_tri = indexOf(vi, tri);
        v_left = tri.verts[prev(index_in_tri)];
        v_right = tri.verts[next(index_in_tri)];
        //! find first direction of walk by looking at triangles that contain vi;
        while (!segmentsIntersectOrTouch(v_left, v_right, vi, vj))
        {
            tri_ind = tri.neighbours[index_in_tri];
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(vi, tri);

            v_left = v_right;
            v_right = tri.verts[next(index_in_tri)];
        }

        intersected_edges.push_back({v_left, v_right});
        intersected_tri_inds.push_back({tri_ind});

        auto v_current = tri.verts[next(index_in_tri)];
        tri_ind = tri.neighbours[next(index_in_tri)];
        EdgeI<Vertex> e_next1;
        EdgeI<Vertex> e_next2;

        //! walk in the found direction to the triangle containing end_ind;
        while (tri_ind != end_tri_ind)
        {
            tri = m_triangles[tri_ind];
            index_in_tri = indexOf(v_current, tri);
            assert(index_in_tri != -1); //! we expect v_current to always exist in tri

            e_next1 = {v_current, tri.verts[next(index_in_tri)]};
            e_next2 = {tri.verts[next(index_in_tri)], tri.verts[prev(index_in_tri)]};
            if (e_next1.from == vj || e_next2.to() == vj || e_next1.to() == vj)
            {
                break; //! we found end_v_ind;
            }
            intersected_tri_inds.push_back(tri_ind);

            if (edgesIntersect(e_next1, e_inserted))
            {
                intersected_edges.push_back(e_next1);
                tri_ind = tri.neighbours[index_in_tri];
            }
            else if (edgesIntersect(e_next2, e_inserted))
            {
                intersected_edges.push_back(e_next2);
                tri_ind = tri.neighbours[next(index_in_tri)];
                v_current = tri.verts[next(index_in_tri)];
            }
            else
            {
                break;
            }
        }
    }

    template <class Vertex>
    bool Triangulation<Vertex>::hasGoodOrientation(const Triangle<Vertex> &tri) const
    {
        return isCounterClockwise(tri.verts[2], tri.verts[1], tri.verts[0]) &&
               isCounterClockwise(tri.verts[0], tri.verts[2], tri.verts[1]) &&
               isCounterClockwise(tri.verts[1], tri.verts[0], tri.verts[2]);
    }

    template <class Vertex>
    bool Triangulation<Vertex>::isCounterClockwise(const Vertex &v_query, const Vertex &v1, const Vertex &v2) const
    {
        // Vertex v21_norm = {-v2.y + v1.y, v2.x - v1.x};
        // dot(v_query - v1, v21_norm) >= 0;
        return orient(v_query, v1, v2) >= 0;
    }

    //! \param v    vertex
    //! \param tri  triangle
    //! \returns index in triangle tri corresponding to vertex \p v
    template <class Vertex>
    int Triangulation<Vertex>::indexOf(const Vertex &v, const Triangle<Vertex> &tri) const
    {
        if (tri.verts[0] == v)
        {
            return 0;
        }
        else if (tri.verts[1] == v)
        {
            return 1;
        }
        else if (tri.verts[2] == v)
        {
            return 2;
        }
        return -1;
    }

    //! \brief vertex \p vp and \p v3 must lie opposite to each other
    //! \param vp
    //! \param v1
    //! \param v2
    //! \param v3
    //! \returns true if quadrilateral formed by four vertices is Delaunay
    template <class Vertex>
    bool Triangulation<Vertex>::needSwap(const Vertex &vp, const Vertex &v1, const Vertex &v2, const Vertex &v3) const
    {

        auto v13 = cdt::Vector2f(v1 - v3);
        auto v23 = cdt::Vector2f(v2 - v3);
        auto v1p = cdt::Vector2f(v1 - vp);
        auto v2p = cdt::Vector2f(v2 - vp);

        const auto cos_a = dot(v13, v23);
        const auto cos_b = dot(v1p, v2p);

        if (cos_a >= 0 && cos_b >= 0)
        {
            return false;
        }
        if (cos_a < 0 && cos_b < 0)
        {
            return true;
        }
        const auto sin_ab = static_cast<float>(v13.x * v23.y - v23.x * v13.y) * cos_b +
                            static_cast<float>(v2p.x * v1p.y - v1p.x * v2p.y) * cos_a;

        if (-sin_ab < 0)
        { //! I HAVE NO CLUE WHY THERE HAS TO BE MINUS AND IT MAKES ME SAD!!! (in the article there is plus!)
            return true;
        }

        return false;
    }

    template <class Vertex>
    void Triangulation<Vertex>::dumpToFile(const std::string filename) const
    {

        std::ofstream file(filename);
        if (file.is_open())
        {
            file << "Vertices:\n";
            for (const auto vertex : m_vertices)
            {
                file << vertex.x << " " << vertex.y << "\n";
            }
            file << "Triangles:\n";
            for (const auto &tri : m_triangles)
            {
                file << (int)tri.neighbours[0] << " " << (int)tri.neighbours[1] << " " << (int)tri.neighbours[2] << "\n";
            }
            file.close();
        }
        else
        {
            //        "say some warning message or something";
        }
    }
    template <class Vertex>
    long long Triangulation<Vertex>::det(const Vertex &v1, const Vertex &v2, const Vertex &v3) const
    {
        long long a = v1.x * v2.y + v2.x * v3.y + v3.x * v1.y;
        long long b = v1.y * v2.x + v2.y * v3.x + v3.y * v1.x;
        return a - b;
    }

    //! \brief checks if there are no degenerate triangles (whose points are colinear)
    template <class Vertex>
    bool Triangulation<Vertex>::allTrianglesValid() const
    {
        for (auto &tri : m_triangles)
        {
            if (det(tri.verts[0], tri.verts[1], tri.verts[2]) == 0)
            {
                return false;
            }
        }
        return true;
    }

    template <class Vertex>
    bool Triangulation<Vertex>::isDelaunay(const Triangle<Vertex> &tri) const
    {
        for (int i = 0; i < 3; ++i)
        {
            auto n_ind = tri.neighbours[i];

            if (n_ind != -1)
            {
                auto v_opp = m_vertices.at(oppositeOfEdge(tri, EdgeI{tri.verts[i], tri.verts[next(i)]}));
                if (needSwap(v_opp, tri.verts[i], tri.verts[next(i)], tri.verts[prev(i)]))
                {
                    return false;
                }
            }
        }
        return true;
    }

    //! \returns true if all triangles satisfy Delaunay property
    template <class Vertex>
    bool Triangulation<Vertex>::allAreDelaunay() const
    {
        for (const auto &tri : m_triangles)
        {
            if (!isDelaunay(tri))
            {
                return false;
            }
        }
        return true;
    }

    //! \brief checks if the given points lie on the same line
    //! \param v1
    //! \param v2
    //! \param v3
    //! \returns true if the points lie on the same line
    template <class Vertex>
    bool Triangulation<Vertex>::areCollinear(const Vertex &v1, const Vertex &v2, const Vertex &v3) const
    {
        return approx_equal_zero(orient(v1, v2, v3));
    }

    //! \brief checks if the point \p v_query lies between points \p v1 and \p v2
    //! \param v_query
    //! \param v1
    //! \param v2
    template <class Vertex>
    bool Triangulation<Vertex>::liesBetween(const Vertex &v_query, const Vertex &v1, const Vertex &v2) const
    {
        return areCollinear(v_query, v1, v2) &&
               dot(v_query - v1, v2 - v1) * dot(v_query - v2, v2 - v1) <= 0;
    }

    //! \brief fixes triangles around \p new_vertex not satisfying the Delaunay property by flipping edges
    //! \param  new_vertex
    //! \param  triangles_to_fix pairs of triangle inds with
    template <class Vertex>
    void Triangulation<Vertex>::fixDelaunayProperty(Vertex new_vertex,
                                                    std::stack<std::pair<TriInd, TriInd>> &triangles_to_fix)
    {
        //! fix delaunay
        while (!triangles_to_fix.empty())
        {
            auto old_tri_ind = triangles_to_fix.top().first;
            auto next_tri_ind = triangles_to_fix.top().second;
            triangles_to_fix.pop();
            if (next_tri_ind == -1)
            {
                continue;
            }

            auto &old_tri = m_triangles[old_tri_ind];
            auto &next_tri = m_triangles[next_tri_ind];

            auto opposite_ind_in_tri = oppositeIndex(old_tri_ind, next_tri);
            auto new_vert_ind_in_tri = indexOf(new_vertex, old_tri);
            auto v3 = next_tri.verts[opposite_ind_in_tri];
            auto v1 = next_tri.verts[next(opposite_ind_in_tri)];
            auto v2 = next_tri.verts[prev(opposite_ind_in_tri)];
            auto vp = new_vertex;

            if (needSwap(vp, v1, v2, v3) && !next_tri.is_constrained[next(opposite_ind_in_tri)])
            {

                auto &a = old_tri.verts[prev(new_vert_ind_in_tri)];
                assert(isCounterClockwise(a, v3, vp));
                swapConnectingEdgeClockwise(old_tri_ind, next_tri_ind);

                triangles_to_fix.emplace(old_tri_ind, old_tri.neighbours[next(new_vert_ind_in_tri)]);
                triangles_to_fix.emplace(next_tri_ind, next_tri.neighbours[prev(opposite_ind_in_tri)]);
            }
        }
        assert(allTrianglesValid());
    }

    template <class Vertex>
    template <class  VectorType>
    bool Triangulation<Vertex>::isWithinBoundary(const VectorType &query)
    {
        return query.x >= 0 && query.x <= m_boundary.x &&
               query.y >= 0 && query.y <= m_boundary.y;
    }

    template class Triangulation<cdt::Vector2<int>>;
    template class Triangulation<cdt::Vector2<unsigned int>>;
    template class Triangulation<cdt::Vector2<unsigned short>>;
    template class Triangulation<cdt::Vector2<unsigned char>>;
    template class Triangulation<cdt::Vector2<float>>;

} // namespace cdt