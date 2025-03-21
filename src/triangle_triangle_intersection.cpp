#include "triangle_triangle_intersection.h"
#include "Ray.h"
#include "ray_intersect_triangle.h"

bool triangle_triangle_intersection(
  const Eigen::RowVector3d & A0,
  const Eigen::RowVector3d & A1,
  const Eigen::RowVector3d & A2,
  const Eigen::RowVector3d & B0,
  const Eigen::RowVector3d & B1,
  const Eigen::RowVector3d & B2)
{
  // triangle A intersects triangle B iff at least one of the edges from one of the triangle intersects the other triangle
  
  // we will construct rays going around the triangle
  Ray a0_ray(A0, A1 - A0); Ray a1_ray(A1, A2 - A1); Ray a2_ray(A2, A0 - A2);
  // we only want hits that go from the start to the end of the triangle ray, so we restrict as follows
  double min_t = 0, max_t = 1, t;
  bool a0_ray_hit_B = ray_intersect_triangle(a0_ray, B0, B1, B2, min_t, max_t, t);
  bool a1_ray_hit_B = ray_intersect_triangle(a1_ray, B0, B1, B2, min_t, max_t, t);
  bool a2_ray_hit_B = ray_intersect_triangle(a2_ray, B0, B1, B2, min_t, max_t, t);

  // same for B
  Ray b0_ray(B0, B1 - B0); Ray b1_ray(B1, B2 - B1); Ray b2_ray(B2, B0 - B2);
  bool b0_ray_hit_A = ray_intersect_triangle(b0_ray, A0, A1, A2, min_t, max_t, t);
  bool b1_ray_hit_A = ray_intersect_triangle(b1_ray, A0, A1, A2, min_t, max_t, t);
  bool b2_ray_hit_A = ray_intersect_triangle(b2_ray, A0, A1, A2, min_t, max_t, t);

  return a0_ray_hit_B or a1_ray_hit_B or a2_ray_hit_B or b0_ray_hit_A or b1_ray_hit_A or b2_ray_hit_A;
}
