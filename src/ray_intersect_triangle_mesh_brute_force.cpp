#include "ray_intersect_triangle_mesh_brute_force.h"
#include "ray_intersect_triangle.h"
#include <limits>

bool ray_intersect_triangle_mesh_brute_force(
  const Ray & ray,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const double min_t,
  const double max_t,
  double & hit_t,
  int & hit_f)
{
  auto triangle_vertex_position_matrix = V;
  auto triangle_indices_matrix = F;
  bool hit_at_least_one_thing = false;
  double curr_t;
  double min_t_so_far = std::numeric_limits<double>::infinity();

  // each row of the matrix represents a single triangle
  int num_triangles = triangle_indices_matrix.rows();

  for (int triangle_idx = 0; triangle_idx < num_triangles; triangle_idx++) {
    int index_a = triangle_indices_matrix(triangle_idx, 0);
    int index_b = triangle_indices_matrix(triangle_idx, 1);
    int index_c = triangle_indices_matrix(triangle_idx, 2);

    auto vertex_pos_a = triangle_vertex_position_matrix.row(index_a);
    auto vertex_pos_b = triangle_vertex_position_matrix.row(index_b);
    auto vertex_pos_c = triangle_vertex_position_matrix.row(index_c);

    bool ray_hit_something = ray_intersect_triangle(ray, vertex_pos_a, vertex_pos_b, vertex_pos_c, min_t, max_t, curr_t);

    if (ray_hit_something && curr_t < min_t_so_far) {
      hit_at_least_one_thing = true;
      min_t_so_far = curr_t;
      hit_f = triangle_idx;
    }
  }

  hit_t = min_t_so_far;

  return hit_at_least_one_thing;
}
