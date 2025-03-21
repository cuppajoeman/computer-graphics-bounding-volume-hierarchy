#include "ray_intersect_triangle.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

// precond: det is non-zero
Eigen::Vector3d cramers_rule(const Eigen::Matrix3d& A, 
                             const Eigen::Vector3d& b) {

  assert(A.determinant() != 0);

  auto detA = A.determinant();

  Eigen::Vector3d x;

  Eigen::Matrix3d A1 = A;
  A1.col(0) = b;
  x(0) = A1.determinant() / detA;

  Eigen::Matrix3d A2 = A;
  A2.col(1) = b;
  x(1) = A2.determinant() / detA;

  Eigen::Matrix3d A3 = A;
  A3.col(2) = b;
  x(2) = A3.determinant() / detA;

  return x;
}

bool ray_intersect_triangle(
  const Ray & ray,
  const Eigen::RowVector3d & A,
  const Eigen::RowVector3d & B,
  const Eigen::RowVector3d & C,
  const double min_t,
  const double max_t,
  double & t)
{

  Eigen::Vector3d pa = A.transpose();
  Eigen::Vector3d pb = B.transpose();
  Eigen::Vector3d pc = C.transpose();

  auto d1 = pa - pb;
  auto d2 = pa - pc;

  auto b = pa - ray.origin;


  Eigen::Matrix3d M;
  M << d1, d2, ray.direction;

  double detM = M.determinant();

  if (detM == 0) {
    return false;
  }

  auto solution = cramers_rule(M, b);

  auto beta = solution.x();
  auto gamma = solution.y();
  auto temp_t = solution.z();
  
  if (temp_t < min_t || temp_t > max_t || gamma < 0 || gamma > 1 || beta < 0 || beta > (1-gamma)){
    return false;  
  } 

  t = temp_t;  
  return true;
}

