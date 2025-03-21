#include "point_AABBTree_squared_distance.h"
#include "point_box_squared_distance.h"
#include <limits>
#include <memory>
#include <queue> // std::priority_queue

bool point_AABBTree_squared_distance(
    const Eigen::RowVector3d & query,
    const std::shared_ptr<AABBTree> & root,
    const double min_sqrd,
    const double max_sqrd,
    double & sqrd,
    std::shared_ptr<Object> & descendant)
{
  // given a distance to tree map we compare by only comparing their distances.
  auto comparison_fun = [](std::pair<double, std::shared_ptr<AABBTree>> a, std::pair<double, std::shared_ptr<AABBTree>> b) {
    return a.first > b.first;
  };
  std::priority_queue<std::pair<double, std::shared_ptr<AABBTree>>, std::vector<std::pair<double, std::shared_ptr<AABBTree>>>, decltype(comparison_fun)> pq(comparison_fun);

  sqrd = std::numeric_limits<double>::infinity();
  double squared_distance_to_box = point_box_squared_distance(query, root->box);
  pq.emplace(squared_distance_to_box, root);

  // just your standard bfs
  while (not pq.empty()) {
    auto popped_elt = pq.top();
    double distance_to_subtrees_bounding_box = popped_elt.first;
    std::shared_ptr<AABBTree> subtree = popped_elt.second;
    pq.pop();

    // we only care if a bounding box is closer than one we've seen so far
    if (distance_to_subtrees_bounding_box < sqrd) { 
      if (subtree->num_leaves <= 2) { // then this is a leaf
        double subtree_sqrd;
        std::shared_ptr<Object> subtree_descendant;

        // we don't have to check for left to exist because we always put one on the left, while constructing the tree
        bool valid_distance_recorded = subtree->left->point_squared_distance(query, min_sqrd, max_sqrd, subtree_sqrd, subtree_descendant);
        if (valid_distance_recorded and subtree_sqrd < sqrd) {
          sqrd = subtree_sqrd;
          descendant = subtree->left;
        }

        bool right_exists = subtree->right != nullptr;
        if (right_exists) { // this occurs because when constructing a tree with one child node it goes to the left
          valid_distance_recorded = subtree->right->point_squared_distance(query, min_sqrd, max_sqrd, subtree_sqrd, subtree_descendant);
          if (valid_distance_recorded and subtree_sqrd < sqrd) {
            sqrd = subtree_sqrd;
            descendant = subtree->right;
          }
        }
      } else { // we are not at a leaf, so recurse, ie add stuff back onto the queue and keep going.
        double squared_distance_to_left_bounding_box = point_box_squared_distance(query, subtree->left->box);
        // have to cast here because left is just an object, and we need to be sure to say that its a tree
        // it is guarenteed to be a tree because we are not a leaf!
        pq.emplace(squared_distance_to_left_bounding_box, std::dynamic_pointer_cast<AABBTree>(subtree->left));
        // add the right too.
        double squared_distance_to_right_bounding_box = point_box_squared_distance(query, subtree->right->box);
        pq.emplace(squared_distance_to_right_bounding_box, std::dynamic_pointer_cast<AABBTree>(subtree->right));
      }
    }
  }
  // return if a minimal distance was found
  return descendant != nullptr;
}
