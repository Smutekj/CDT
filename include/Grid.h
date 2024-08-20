#pragma once
#include <stdexcept>
#include "Utils/Vector2.h"

namespace cdt
{

  class Grid
  {

  public:
    utils::Vector2i m_cell_count;
    utils::Vector2f m_cell_size;

  public:
    Grid(utils::Vector2i n_cells, utils::Vector2f box_size);

    [[nodiscard]] size_t coordToCell(float x, float y) const;
    [[nodiscard]] size_t coordToCell(utils::Vector2f r) const;
    // [[nodiscard]] size_t coordToCell(utils::Vector2i r_coord) const;
    [[nodiscard]] size_t cellIndex(int ix, int iy) const;
    [[nodiscard]] size_t cellIndex(utils::Vector2i) const;

    [[nodiscard]] size_t cellCoordX(size_t cell_index) const;
    [[nodiscard]] size_t cellCoordY(size_t cell_index) const;

    [[nodiscard]] size_t cellCoordX(utils::Vector2f r_coord) const;
    [[nodiscard]] size_t cellCoordY(utils::Vector2f r_coord) const;

    [[nodiscard]] utils::Vector2i cellCoords(utils::Vector2f r_coord) const;
    [[nodiscard]] utils::Vector2i cellCoords(utils::Vector2i r_coord) const;
    [[nodiscard]] utils::Vector2i cellCoords(size_t cell_index) const;

    size_t getNCells() const;
  };

} // namespace cdt;