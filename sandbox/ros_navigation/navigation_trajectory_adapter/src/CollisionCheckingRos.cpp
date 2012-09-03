/******************************************************************************
 * Copyright (c) 2011
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of GPS GmbH nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "navigation_trajectory_adapter/CollisionCheckingRos.h"
#include "navigation_trajectory_adapter/PathUtilities.h"
#include "navigation_trajectory_adapter/PathIterator.h"
#include "navigation_trajectory_adapter/LinearInterpolation.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Point.h>

CollisionCheckingRos::CollisionCheckingRos(costmap_2d::Costmap2DROS* costmap) : costmap(costmap) {
}

CollisionCheckingRos::CollisionCheckingRos(CollisionCheckingRos& orig) : costmap(orig.costmap) {

}


CollisionCheckingRos::~CollisionCheckingRos() {

}

bool CollisionCheckingRos::check(const std::vector <FrameWithId>& path, const FrameWithId& actualPose) {
    return false;
}

bool CollisionCheckingRos::collisionCheck(const std::vector <FrameWithId>& path,
         const FrameWithId& actualPose, double interpolationStep, unsigned int numberOfSteps) {
    
    costmap_2d::Costmap2D costmapCopy;
    std::vector <geometry_msgs::Point> orientedFootprint;
    
    costmap->getOrientedFootprint(orientedFootprint);
    double circumscribedRadius = costmap->getCircumscribedRadius();
    double inscribedRadius = costmap->getInscribedRadius();
    
    costmap->clearRobotFootprint();
    costmap->getCostmapCopy(costmapCopy);
    base_local_planner::CostmapModel collisionChecker(costmapCopy);
    
    std::vector <FrameWithId> prunedPath;
    prunePath(path, actualPose, prunedPath);

    const double step = 0.1;
    
    LinearInterpolation interpolator;
    std::vector <FrameWithId> interpolatedPath;
    interpolator.interpolate(prunedPath, interpolatedPath, step);
    PathIterator it(prunedPath);
    
    while (it.hasNext()) {
        
        const FrameWithId& frame = it.next();
        
    }
    
    /*for (int i = 0; i < trajectory.trajectory.size(); i++) {
        nav_msgs::Odometry odom = trajectory.trajectory[i];
        geometry_msgs::Pose pose = odom.pose.pose;

        /* *
         * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
         * @param  position The position of the robot in world coordinates
         * @param  footprint The specification of the footprint of the robot in world coordinates
         * @param  inscribed_radius The radius of the inscribed circle of the robot
         * @param  circumscribed_radius The radius of the circumscribed circle of the robot
         * @return Positive if all the points lie outside the footprint, negative otherwise
         */

  //      geometry_msgs::Point position = pose.position;


  /*      vector <geometry_msgs::Point> orientedFootprint;
        costmap->getOrientedFootprint(orientedFootprint);
        double circumscribedRadius = costmap->getCircumscribedRadius();
        double inscribedRadius = costmap->getInscribedRadius();

        double collision = collisionChecker.footprintCost(position,
                orientedFootprint,
                inscribedRadius,
                circumscribedRadius);

        if (collision < 0) {
            //ROS_INFO("Collision at pose: %f, %f", pose.position.x, pose.position.y);
            return false;
        }

    }
   * */


    return true;
}