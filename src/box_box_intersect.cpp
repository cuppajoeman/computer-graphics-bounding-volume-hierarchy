#include "box_box_intersect.h"
bool box_box_intersect(
  const BoundingBox & A,
  const BoundingBox & B)
{
  for (int i = 0; i < 3; i++) {
    double mA = A.min_corner(i); double MA = A.max_corner(i);
    double mB = B.min_corner(i); double MB = B.max_corner(i);

    /*
     * Ways intersection can happen, first we have this:
      *                                                     
      *           mB                   MB                   
      *            |                    |                   
      *                       |                             
      *                       mA                            
      *                                                     
      *                                                     
     * If that didn't occur, then the only other way an intersection can occur is
      *                                                     
      *                                                     
      *                                                     
      *           mB                   MB                   
      *            |                    |                   
      *      |               |             
      *      mA             MA             
      *                                                     
     * As if MB < mA no intersection happens, but also note the above is equiavlent to
     * ma <= mB <= mA, 
      *                                                     
      */

    // by the above the only ways an intersection can occur are as follows:
    bool case1 = mB <= mA && mA <= MB;
    bool case2 = mA <= mB && mB <= MA;
    
    if (not (case1 || case2)) {
      return false;
    }
  }
  // you only got here if it was true for all
  return true;

}

