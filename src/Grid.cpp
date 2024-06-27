

#include <cmath>
#include <cassert>
#include <iostream>

#include "../include/Grid.h"

namespace cdt{

//! \brief computes cell_index of the cell containing given point
//! \param x 
//! \param y
//! \returns cell index
size_t Grid::coordToCell(const float x, const float y) const { return coordToCell({x, y}); }

//! \brief computes cell_index of the cell containing given point
//! \param r_coords given point 
//! \returns cell index
size_t Grid::coordToCell(const cdt::Vector2f r_coord) const {
    const size_t ix = static_cast<size_t>(std::floor(r_coord.x / m_cell_size.x)) % m_cell_count.x;
    const size_t iy = static_cast<size_t>(std::floor(r_coord.y / m_cell_size.y)) % m_cell_count.y;

    assert(ix + iy * m_cell_count.x < m_cell_count.x * m_cell_count.y);
    return ix + iy * m_cell_count.x;
}

//! \brief computes cell_index of the cell corresponding to given 2D cell coordinates
//! \param ix
//! \param iy 
//! \returns cell index
[[nodiscard]] size_t Grid::cellIndex(const int ix, const int iy) const { return cellIndex({ix, iy}); }

//! \brief computes cell_index of the cell corresponding to given 2D cell coordinates
//! \param ixy given 2D cell coordinates
//! \returns cell index
[[nodiscard]] size_t Grid::cellIndex(cdt::Vector2i ixy) const { return ixy.x + ixy.y * m_cell_count.x; }

//! \brief computes the x component of 2D cell coordinates 
//! \param cell_index
//! \returns x component of 2D cell coordinates
size_t Grid::cellCoordX(const size_t cell_index) const { return cell_index % m_cell_count.x; }

//! \brief computes the y component of 2D cell coordinates 
//! \param cell_index
//! \returns y component of 2D cell coordinates
size_t Grid::cellCoordY(const size_t cell_index) const { return (cell_index / m_cell_count.x) % m_cell_count.y; }

//! \brief computes the 2D cell coordinates of a given cell 
//! \param cell_index
//! \returns 2D cell coordinates
cdt::Vector2i Grid::cellCoords(const size_t cell_index) const {
    return {static_cast<int>(cellCoordX(cell_index)), static_cast<int>(cellCoordY(cell_index))};
}

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns x component of 2D cell coordinates
size_t Grid::cellCoordX(const cdt::Vector2f r_coord) const { return static_cast<size_t>(r_coord.x / m_cell_size.x); }

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns y component of 2D cell coordinates
size_t Grid::cellCoordY(const cdt::Vector2f r_coord) const { return static_cast<size_t>(r_coord.y / m_cell_size.y); }

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns 2D cell coordinates
cdt::Vector2i Grid::cellCoords(const cdt::Vector2f r_coord) const {
    const auto cell_coord_x = static_cast<int>(r_coord.x / m_cell_size.x);
    const auto cell_coord_y = static_cast<int>(r_coord.y / m_cell_size.y);
    return {cell_coord_x, cell_coord_y};
}

//! \brief computes the 2D cell coordinates of cell containing the given point 
//! \param r_coord given point
//! \returns 2D cell coordinates
cdt::Vector2i Grid::cellCoords(const cdt::Vector2i r_coord) const {
    const auto cell_coord_x = static_cast<int>(r_coord.x / m_cell_size.x);
    const auto cell_coord_y = static_cast<int>(r_coord.y / m_cell_size.y);
    return {cell_coord_x, cell_coord_y};}


Grid::Grid(cdt::Vector2i n_cells, cdt::Vector2f box_size)
    : m_cell_count(n_cells)
    , m_cell_size(box_size.x/n_cells.x, box_size.y/n_cells.y) {}


size_t Grid::getNCells() const{
    return m_cell_count.x*m_cell_count.y;
}

} // namespace cdt