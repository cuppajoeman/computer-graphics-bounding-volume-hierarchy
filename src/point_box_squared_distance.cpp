#include "point_box_squared_distance.h"

double point_box_squared_distance(
  const Eigen::RowVector3d & query,
  const BoundingBox & box)
{
  // given two points in R3 (x1, y1, z1), (x2, y2, z2), then the squared distance is just x1 * x2 + y1 * y2 + z1 * x2
  // but now we have to compute that to the box so consider on one axis first, 
  /*
   *                bMin              bMax               
   *                 |                  |                
   * case 1:   q                                         
   * then bMin - q > 0 and q - bMax < 0
   * we want dist = bMin - q in this case
   *                                                     
   *
   *                bMin              bMax               
   *                 |                  |                
   * case 2:                 q                           
   * then bMin - q < 0 and q - bMax < 0
   * we want dist = 0 in this case
   *                                                     
   *
   *                bMin              bMax               
   *                 |                  |                
   * case 3:                                  q          
   * then bMin - q < 0 and q - bMax < 0
   * we want dist = q - bMax in this case
   *                                                     
   */
  // due to the above we can deduce the formula as 
  // max(bMin - q, q - bMax, 0)
  double square_distance = 0;
  for (int axis_idx = 0; axis_idx < 3; axis_idx++) {
    // using the crafty formula derived above.
    double curr_axis_dist = std::max(std::max(box.min_corner(axis_idx) - query(axis_idx), query(axis_idx) - box.max_corner(axis_idx)), 0.0);
    square_distance += curr_axis_dist * curr_axis_dist;
  }
  return square_distance;
}
