/* 
 * File:   CosllisionCheckingRos.h
 * Author: alexey
 *
 * Created on September 1, 2012, 2:19 PM
 */

#ifndef COLLISIONCHECKINGROS_H
#define	COLLISIONCHECKINGROS_H

#include "navigation_trajectory_adapter/CollisionChecking.h"
#include "navigation_trajectory_adapter/CollisionChecking.h"

namespace costmap_2d {
    class Costmap2DROS;
}

class CollisionCheckingRos : public CollisionChecking {
public:
    CollisionCheckingRos(const costmap_2d::Costmap2DROS* costmap);
    CollisionCheckingRos(const CollisionCheckingRos& orig);
    virtual ~CollisionCheckingRos();
    
    bool check(const std::vector <FrameWithId>& path, const FrameWithId& actualPose);
    
private:
    
   const costmap_2d::Costmap2DROS* costmap;

};

#endif	/* COLLISIONCHECKINGROS_H */

