#include "insert_box_into_box.h"

void insert_box_into_box(
  const BoundingBox & A,
  BoundingBox & B)
{
  // To "expand" we need to pull things out as far as possible given the extents
  // that equates to taking the min of the mins and the max of the maxes to contain everything
  for (int i = 0; i < 3; i++) {
    auto &aMc = A.max_corner(i); auto &bMc = B.max_corner(i);
    auto &amc = A.min_corner(i); auto &bmc = B.min_corner(i);
    bMc = std::max(bMc, aMc);
    bmc = std::min(bmc, amc);
  }
}

