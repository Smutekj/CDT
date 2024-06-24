#pragma once
#include <stdexcept>
#include "Vector2.h"

namespace cdt
{

  class Grid
  {

  public:
    cdt::Vector2i m_cell_count;
    cdt::Vector2f m_cell_size;

  public:
    Grid(cdt::Vector2i n_cells, cdt::Vector2f box_size);

    [[nodiscard]] size_t coordToCell(float x, float y) const;
    [[nodiscard]] size_t coordToCell(cdt::Vector2f r) const;
    // [[nodiscard]] size_t coordToCell(cdt::Vector2i r_coord) const;
    [[nodiscard]] size_t cellIndex(int ix, int iy) const;
    [[nodiscard]] size_t cellIndex(cdt::Vector2i) const;

    [[nodiscard]] size_t cellCoordX(size_t cell_index) const;
    [[nodiscard]] size_t cellCoordY(size_t cell_index) const;

    [[nodiscard]] size_t cellCoordX(cdt::Vector2f r_coord) const;
    [[nodiscard]] size_t cellCoordY(cdt::Vector2f r_coord) const;

    [[nodiscard]] cdt::Vector2i cellCoords(cdt::Vector2f r_coord) const;
    [[nodiscard]] cdt::Vector2i cellCoords(cdt::Vector2i r_coord) const;
    [[nodiscard]] cdt::Vector2i cellCoords(size_t cell_index) const;

    size_t getNCells() const;
  };

} // namespace cdt;