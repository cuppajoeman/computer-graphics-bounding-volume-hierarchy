#include "AABBTree.h"
#include "insert_box_into_box.h"

AABBTree::AABBTree(
  const std::vector<std::shared_ptr<Object> > & objects,
  int a_depth): 
  depth(std::move(a_depth)), 
  num_leaves(objects.size())
{
  int num_objects = objects.size();

  if (num_objects == 0) {
    this->left = nullptr;
    this->right = nullptr;
  } else if (num_objects == 1) {
    // if there is one object we aribtrarily choose to put it in the left
    this->left = objects[0];
    // nothing on the right
    this->right = nullptr;
    insert_box_into_box(this->left->box, this->box);
  } else if (num_objects == 2) {
    // just do a simple split if there's two
    this->left = objects[0];
    this->right = objects[1];
    // expand our box to contain the boxes of our left and right children
    insert_box_into_box(this->right->box, this->box);
    insert_box_into_box(this->left->box, this->box);
  } else { // more than two objects, so we use the midpoint alg now
    // first expand our box to contain all child boxes
    for (auto &object: objects) {
      insert_box_into_box(object->box, this->box);
    }

    // then located the longest axis of the box we just stuffed our children into 
    // (don't call child protection services, it's alright)
    int longest_axis; // 0, 1, 2, <-> x, y, z
    double longest_axis_length_so_far = -1; // all lenghts will be >= 0
    for (int axis_idx = 0; axis_idx < 3; axis_idx ++) {
      double curr_axis_length = this->box.max_corner(axis_idx) - this->box.min_corner(axis_idx);
      if (curr_axis_length > longest_axis_length_so_far) {
        longest_axis_length_so_far = curr_axis_length;
        longest_axis = axis_idx;
      }
    }

    double midpoint = (this->box.max_corner(longest_axis) + this->box.min_corner(longest_axis)) / 2;

    // now split based on which side the center of each object is wrt midpoint
    std::vector<std::shared_ptr<Object>> objects_left_of_mid_point;
    std::vector<std::shared_ptr<Object>> objects_right_or_equal_to_mid_point;
    for (auto &object : objects) {
      if (object->box.center()[longest_axis] <= midpoint) {
        objects_left_of_mid_point.emplace_back(object);
      } else {
        objects_right_or_equal_to_mid_point.emplace_back(object);
      }
    }

    // balance the tree so that empty subtrees become non-empty. also we know that there are at least
    // three elements in this branch, and either both sides have objects, or one doesn't and there are at least 3 on the other side

    int exactly_one_of_the_following_are_true = 0;

    if (objects_left_of_mid_point.size() >= 1 and objects_right_or_equal_to_mid_point.size() >= 1) {
      exactly_one_of_the_following_are_true ++;
    }
    if (objects_left_of_mid_point.size() == 0 and objects_right_or_equal_to_mid_point.size() >= 2) {
      objects_left_of_mid_point.emplace_back(objects_right_or_equal_to_mid_point.back());
      objects_right_or_equal_to_mid_point.pop_back();
      exactly_one_of_the_following_are_true ++;
    }
    if (objects_left_of_mid_point.size() >= 2 and objects_right_or_equal_to_mid_point.size() == 0) {
      objects_right_or_equal_to_mid_point.emplace_back(objects_left_of_mid_point.back());
      objects_left_of_mid_point.pop_back();
      exactly_one_of_the_following_are_true ++;
    }

    // this being true enforces the fact that left and right will have at least one element
    assert(exactly_one_of_the_following_are_true == 1);

    this->left = std::make_shared<AABBTree>(objects_left_of_mid_point, a_depth + 1);
    this->right = std::make_shared<AABBTree>(objects_right_or_equal_to_mid_point, a_depth + 1);
  }


}
