#include "find_all_intersecting_pairs_using_AABBTrees.h"
#include "box_box_intersect.h"
// Hint: use a list as a queue
#include <list> 
#include <memory>

void find_all_intersecting_pairs_using_AABBTrees(
  const std::shared_ptr<AABBTree> & rootA,
  const std::shared_ptr<AABBTree> & rootB,
  std::vector<std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > > & 
    leaf_pairs)
{
  std::list<std::pair<std::shared_ptr<AABBTree>, std::shared_ptr<AABBTree>>> q;
  if (box_box_intersect(rootA->box, rootB->box)) {
    q.emplace_back(rootA, rootB);
  }

  std::shared_ptr<AABBTree> nodeA, nodeB;
  while (not q.empty()) {
    nodeA = q.front().first;
    nodeB = q.front().second;
    q.pop_front();

    bool node_a_and_b_are_leaves = nodeA->num_leaves <= 2 and nodeB->num_leaves <= 2;

    // the following code looks dense and complex, but its not it's just book-keeping and case work.
    if (node_a_and_b_are_leaves) {
      if (nodeA->left and nodeB->left and box_box_intersect(nodeA->left->box, nodeB->left->box)) {
        leaf_pairs.emplace_back(nodeA->left, nodeB->left);
      }
      if (nodeA->left and nodeB->right and box_box_intersect(nodeA->left->box, nodeB->right->box)) {
        leaf_pairs.emplace_back(nodeA->left, nodeB->right);
      }
      if (nodeA->right and nodeB->left and box_box_intersect(nodeA->right->box, nodeB->left->box)) {
        leaf_pairs.emplace_back(nodeA->right, nodeB->left);
      }
      if (nodeA->right and nodeB->right and box_box_intersect(nodeA->right->box, nodeB->right->box)) {
        leaf_pairs.emplace_back(nodeA->right, nodeB->right);
      }
    } else if (nodeA->num_leaves <= 2) { // and nodeB is not a leaf
      // all things that look unsafe here are safe because nodeB is not a leaf!
      if (box_box_intersect(nodeA->box, nodeB->left->box)) { 
        q.emplace_back(nodeA, std::dynamic_pointer_cast<AABBTree>(nodeB->left));
      }
      if (box_box_intersect(nodeA->box, nodeB->right->box)) { 
        q.emplace_back(nodeA, std::dynamic_pointer_cast<AABBTree>(nodeB->right));
      }
    } else if (nodeB->num_leaves <= 2) { // and nodeA is not a leaf
      if (box_box_intersect(nodeA->left->box, nodeB->box)) { 
        q.emplace_back( std::dynamic_pointer_cast<AABBTree>(nodeA->left), nodeB);
      }
      if (box_box_intersect(nodeA->right->box, nodeB->box)) { 
        q.emplace_back( std::dynamic_pointer_cast<AABBTree>(nodeA->right), nodeB);
      }
    } else { // both are not leaves
      if (box_box_intersect(nodeA->left->box, nodeB->left->box)) {
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(nodeA->left), std::dynamic_pointer_cast<AABBTree>(nodeB->left));
      }
      if (box_box_intersect(nodeA->left->box, nodeB->right->box)) {
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(nodeA->left), std::dynamic_pointer_cast<AABBTree>(nodeB->right));
      }
      if (box_box_intersect(nodeA->right->box, nodeB->left->box)) {
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(nodeA->right), std::dynamic_pointer_cast<AABBTree>(nodeB->left));
      }
      if (box_box_intersect(nodeA->right->box, nodeB->right->box)) {
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(nodeA->right), std::dynamic_pointer_cast<AABBTree>(nodeB->right));
      }
    }
  }

}
