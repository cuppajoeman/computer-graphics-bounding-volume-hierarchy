#include "ray_intersect_box.h"
#include <iostream>

bool ray_intersect_box(
  const Ray & ray,
  const BoundingBox& box,
  const double min_t,
  const double max_t)
{
  auto origin_x = ray.origin.x(); auto origin_y = ray.origin.y(); auto origin_z = ray.origin.z();
  auto dir_x = ray.direction.x(); auto dir_y = ray.direction.y(); auto dir_z = ray.direction.z();

  auto min_corner_x = box.min_corner.x(); auto min_corner_y = box.min_corner.y(); auto min_corner_z = box.min_corner.z();
  auto max_corner_x = box.max_corner.x(); auto max_corner_y = box.max_corner.y(); auto max_corner_z = box.max_corner.z();

  // the equation ox + t * dx = p where t is the only variable when solved yields the value 
  // for t such that the x component of the line defined by the equation of the ray equals the value p
  // therefore we can isolate for t to find the value.
 
  // precompute some values in the thing 
  double rx = 1 / dir_x; double ry = 1 / dir_y; double rz = 1 / dir_z;
  double parametric_distance_at_which_the_x_component_of_the_ray_equals_min_corner_x;
  double parametric_distance_at_which_the_x_component_of_the_ray_equals_max_corner_x;
  double parametric_distance_at_which_the_y_component_of_the_ray_equals_min_corner_y;
  double parametric_distance_at_which_the_y_component_of_the_ray_equals_max_corner_y;
  double parametric_distance_at_which_the_z_component_of_the_ray_equals_min_corner_z;
  double parametric_distance_at_which_the_z_component_of_the_ray_equals_max_corner_z;
  
  if (rx >= 0) {
    parametric_distance_at_which_the_x_component_of_the_ray_equals_min_corner_x = (min_corner_x - origin_x) * rx;
    parametric_distance_at_which_the_x_component_of_the_ray_equals_max_corner_x = (max_corner_x - origin_x) * rx;
  } else {
    parametric_distance_at_which_the_x_component_of_the_ray_equals_max_corner_x = (min_corner_x - origin_x) * rx;
    parametric_distance_at_which_the_x_component_of_the_ray_equals_min_corner_x = (max_corner_x - origin_x) * rx;
  }

  if (ry >= 0) {
    parametric_distance_at_which_the_y_component_of_the_ray_equals_min_corner_y = (min_corner_y - origin_y) * ry;
    parametric_distance_at_which_the_y_component_of_the_ray_equals_max_corner_y = (max_corner_y - origin_y) * ry;
  } else {
    parametric_distance_at_which_the_y_component_of_the_ray_equals_max_corner_y = (min_corner_y - origin_y) * ry;
    parametric_distance_at_which_the_y_component_of_the_ray_equals_min_corner_y = (max_corner_y - origin_y) * ry;
  }

  if (rz >= 0) {
    parametric_distance_at_which_the_z_component_of_the_ray_equals_min_corner_z = (min_corner_z - origin_z) * rz;
    parametric_distance_at_which_the_z_component_of_the_ray_equals_max_corner_z = (max_corner_z - origin_z) * rz;
  } else {
    parametric_distance_at_which_the_z_component_of_the_ray_equals_max_corner_z = (min_corner_z - origin_z) * rz;
    parametric_distance_at_which_the_z_component_of_the_ray_equals_min_corner_z = (max_corner_z - origin_z) * rz;
  }

  /*
   *                     |                     |                        
   *                     |                     |                        
   *                     |                     |                        
   *   NOT HIT AREA      |                     |                        
   *                     |                     |                        
   *                     |                     |                        
   *                     |                     |                        
   * -------------------------------------------  ymax -----------------
   *                     |                     |                        
   *                     |                     |                        
   *                     |                     |                        
   *                     |                     |                        
   *                     |  AABB OF INTEREST   |                        
   *                     |                     |                        
   *                     |                     |                        
   *                     |                     |                        
   *                     |                     |                        
   * -------------------------------------------  ymin  ----------------
   *                     |                     |                        
   *                     x                     x                        
   *                --/  m                     m                        
   *             --/     i                     a       NOT HIT AREA     
   *          --/        n                     x                        
   *       --/                                                          
   *    --/                                                             
   *    origin                                                                
   *                                                                    
   *   for a line to go inside the square it must first hit xmin, and then
   *   ymax, if it hit ymax first then x min then it is in the top left area
   *
   *   similarly it must be that it hits ymin and then hit xmax because if
   *   it hit xmax before hitting ymin it would be in the bottom right area
   *                                                                    
   *                                                                    
  */

  // these equations are right because in order to hit the box it needs to at least reach the min extent in each x, y, z component or else it wouldn't be in the box.
  // note that this isn't necessarily a point on the box, if the ray doesn't intersect, hence "potential"
  double parametric_distance_of_potential_entry_point_into_box = std::max(
    std::max( parametric_distance_at_which_the_x_component_of_the_ray_equals_min_corner_x, parametric_distance_at_which_the_y_component_of_the_ray_equals_min_corner_y),
    parametric_distance_at_which_the_z_component_of_the_ray_equals_min_corner_z
  );

  // same thing but with reversed logic to stay in the box.
  double parametric_distance_of_potential_exit_point_out_of_the_box = std::min(
    std::min( parametric_distance_at_which_the_x_component_of_the_ray_equals_max_corner_x, parametric_distance_at_which_the_y_component_of_the_ray_equals_max_corner_y),
    parametric_distance_at_which_the_z_component_of_the_ray_equals_max_corner_z
  );

  bool ray_intersects_box = parametric_distance_of_potential_entry_point_into_box <= parametric_distance_of_potential_exit_point_out_of_the_box;

  if (not ray_intersects_box) {
    return false;
  }

  double min = std::max(min_t, parametric_distance_of_potential_entry_point_into_box);
  double max = std::min(max_t, parametric_distance_of_potential_exit_point_out_of_the_box);

  return min <= max;
}
