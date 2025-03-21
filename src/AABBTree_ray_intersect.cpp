#include "AABBTree.h"

// See AABBTree.h for API
bool AABBTree::ray_intersect(
  const Ray& ray,
  const double min_t,
  const double max_t,
  double & t,
  std::shared_ptr<Object> & descendant) const 
{

  bool ray_hit_our_box = ray_intersect_box(ray, this->box, min_t, max_t);

  if (not ray_hit_our_box) {
    return false; // this one line of code makes the world turn
  }

  std::shared_ptr<Object> object_that_was_hit_inside_of_the_left_subtree,  object_that_was_hit_inside_of_the_right_subtree;
  double parametric_distance_of_hit_in_left_subtree,  parametric_distance_of_hit_in_right_subtree;
  // otherwise the world stops turning (we recurse)

  // short circuit behavior of and makes this safe, also logic is sound because if you don't have a left then you didn't hit anything there!
  bool hit_anything_on_the_left = (this->left != nullptr) and this->left->ray_intersect(ray, min_t, max_t, parametric_distance_of_hit_in_left_subtree, object_that_was_hit_inside_of_the_left_subtree);
  bool hit_anything_on_the_right = (this->right != nullptr) and this->right->ray_intersect(ray, min_t, max_t, parametric_distance_of_hit_in_right_subtree, object_that_was_hit_inside_of_the_right_subtree);

  bool hit_left_subtree_but_missed_all_objects_in_it = hit_anything_on_the_left and object_that_was_hit_inside_of_the_left_subtree == nullptr;
  bool hit_right_subtree_but_missed_all_objects_in_it = hit_anything_on_the_right and object_that_was_hit_inside_of_the_right_subtree == nullptr;

  if (hit_left_subtree_but_missed_all_objects_in_it) {
    // the object that was hit was the aabb tree itself, but nothing else this is an object because it inherits from Object, so this is valid
    object_that_was_hit_inside_of_the_left_subtree = this->left;
  }

  if (hit_right_subtree_but_missed_all_objects_in_it) {
    object_that_was_hit_inside_of_the_right_subtree = this->right;
  }

  if (hit_anything_on_the_left and hit_anything_on_the_right) { // the ray went through both boxes, totally possible (you poke two marshmellows on your campfire stick)
    bool hit_object_in_left_subtree_first = parametric_distance_of_hit_in_left_subtree < parametric_distance_of_hit_in_right_subtree;
    if (hit_object_in_left_subtree_first) {
      t = parametric_distance_of_hit_in_left_subtree;
      descendant = object_that_was_hit_inside_of_the_left_subtree; 
    } else {
      t = parametric_distance_of_hit_in_right_subtree;
      descendant = object_that_was_hit_inside_of_the_right_subtree; 
    }
  } else if (hit_anything_on_the_right) { // and (not hit_anything_on_the_right)
    t = parametric_distance_of_hit_in_right_subtree;
    descendant = object_that_was_hit_inside_of_the_right_subtree;
  } else if (hit_anything_on_the_left) {
    t = parametric_distance_of_hit_in_left_subtree;
    descendant = object_that_was_hit_inside_of_the_left_subtree;
  } else { // nothing hit in right or left, I think redundant based on initial return false, leaving in for redudancy
    return false;
  }

  // if you got here then you hit at least something (because you avoided the early return falses)
  return true;

}

