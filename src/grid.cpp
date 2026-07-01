// #include "grid.h"
// #include "boid.h"
//
// void Grid::move(Boid *unit, double x, double y) {
//   // See which cell it was in.
//   int oldCellX = (int)(unit->x / Grid::CELL_SIZE);
//   int oldCellY = (int)(unit->y / Grid::CELL_SIZE);
//
//   // See which cell it's moving to.
//   int cellX = (int)(x / Grid::CELL_SIZE);
//   int cellY = (int)(y / Grid::CELL_SIZE);
//
//   unit->x = x;
//   unit->y = y;
//   // If it didn't change cells, we're done.
//   if (oldCellX == cellX && oldCellY == cellY)
//     return;
//
//   // Unlink it from the list of its old cell.
//   if (unit->prev_ != NULL) {
//     unit->prev_->next_ = unit->next_;
//   }
//
//   if (unit->next_ != NULL) {
//     unit->next_->prev_ = unit->prev_;
//   }
//
//   // If it's the head of a list, remove it.
//   if (cells_[oldCellX][oldCellY] == unit) {
//     cells_[oldCellX][oldCellY] = unit->next_;
//   }
//
//   // Add it back to the grid at its new cell.
//   add(unit);
// };
//
// void Grid::add(Boid *unit) {
//   // Determine which grid cell it's in.
//   int cellX = (int)(unit->x / Grid::CELL_SIZE);
//   int cellY = (int)(unit->y / Grid::CELL_SIZE);
//
//   // Add to the front of list for the cell it's in.
//   unit->prev_ = NULL;
//   unit->next_ = cells_[cellX][cellY];
//   cells_[cellX][cellY] = unit;
//
//   if (unit->next_ != NULL) {
//     unit->next_->prev_ = unit;
//   }
// }
//
// void Grid::handleCell(Boid *unit) {
//   while (unit != NULL) {
//     Boid *other = unit->next_;
//     while (other != NULL) {
//       if (unit->x == other->x && unit->y == other->y) {
//         // TODO: non so bene cosa scrivere qui
//       }
//       other = other->next_;
//     }
//
//     unit = unit->next_;
//   }
// };
