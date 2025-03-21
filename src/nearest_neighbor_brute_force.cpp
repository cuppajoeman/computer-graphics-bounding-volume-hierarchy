#include "nearest_neighbor_brute_force.h"
#include <limits>// std::numeric_limits<double>::infinity();

void nearest_neighbor_brute_force(
  const Eigen::MatrixXd & points,
  const Eigen::RowVector3d & query,
  int & I,
  double & sqrD)
{
  I = -1;
  sqrD = std::numeric_limits<double>::infinity();

  for (int row_idx = 0; row_idx < points.rows(); row_idx ++) {
    double distance_x_axis =  points(row_idx, 0) - query[0];
    double distance_y_axis =  points(row_idx, 1) - query[1];
    double distance_z_axis =  points(row_idx, 2) - query[2];
    // doing like this for a fast computation
    double squared_dist = distance_x_axis * distance_x_axis + distance_y_axis * distance_y_axis + distance_z_axis * distance_z_axis;
    if (squared_dist < sqrD) {
      sqrD = squared_dist;
      I = row_idx;
    }
  }

}
