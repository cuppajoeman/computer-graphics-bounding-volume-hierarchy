#include "insert_triangle_into_box.h"

void insert_triangle_into_box(
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  BoundingBox & B)
{
  // similar to the box in a box, we want to make sure that the max contins the max of the maxes and the min extents contin the min of the mins
  for (int i = 0; i < 3; i++) {
    auto &bMc = B.max_corner(i); auto &bmc = B.min_corner(i);
    auto pa = a(i); auto pb = b(i); auto pc = c(i);
    bMc = std::max(bMc, std::max(std::max(pa, pb), pc));
    bmc = std::min(bmc, std::min(std::min(pa, pb), pc));
  }
}


