/* 
 * File:   CosllisionCheckingRos.cpp
 * Author: alexey
 * 
 * Created on September 1, 2012, 2:19 PM
 */

#include "navigation_trajectory_adapter/CollisionCheckingRos.h"

#include <costmap_2d/costmap_2d_ros.h>

CollisionCheckingRos::CollisionCheckingRos(const costmap_2d::Costmap2DROS* costmap) : costmap(costmap) {
}

CollisionCheckingRos::CollisionCheckingRos(const CollisionCheckingRos& orig) : costmap(orig.costmap) {
    
}

CollisionCheckingRos::~CollisionCheckingRos() {

}

bool CollisionCheckingRos::check(const std::vector <FrameWithId>& path, const FrameWithId& actualPose) {
    return false;
}
