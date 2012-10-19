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

#include "navigation_trajectory_common/Conversions.h"
#include "navigation_trajectory_common/Utilities.h"
#include "navigation_trajectory_common/Logger.h"

#ifdef DEBUG
#include "navigation_trajectory_common/Stopwatch.h"
#endif

#include "navigation_trajectory_planner/CollisionCheckingRos.h"
#include "navigation_trajectory_planner/LinearInterpolation.h"
#include "navigation_trajectory_planner/PathIterator.h"

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

CollisionCheckingRos::CollisionCheckingRos(costmap_2d::Costmap2DROS* costmap) : costmap(costmap) {
}

CollisionCheckingRos::CollisionCheckingRos(CollisionCheckingRos& orig) : costmap(orig.costmap) {

}

CollisionCheckingRos::~CollisionCheckingRos() {

}

bool CollisionCheckingRos::check(const std::vector <FrameWithId>& path, const FrameWithId& actualPose) {
    bool result = collisionCheck(path, actualPose, 0.1, 100);
    return result;
}

bool CollisionCheckingRos::collisionCheck(const std::vector <FrameWithId>& path,
        const FrameWithId& actualPose, double interpolationStep, unsigned int numberOfSteps) {

    bool collision = false;

    if (path.empty())
        return collision;

    costmap_2d::Costmap2D costmapCopy;
    std::vector <geometry_msgs::Point> orientedFootprint;

    costmap->getOrientedFootprint(orientedFootprint);
    costmap->clearRobotFootprint();
    costmap->getCostmapCopy(costmapCopy);
    base_local_planner::CostmapModel collisionChecker(costmapCopy);

    std::vector <FrameWithId> prunedPath;
    utilities::prunePath(path, actualPose, prunedPath);



    LinearInterpolation interpolator;
    std::vector <FrameWithId> interpolatedPath;
    size_t counter = interpolator.interpolate(prunedPath, interpolatedPath, interpolationStep, numberOfSteps);

#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif


    size_t i;
    for (i = 0; i < counter; ++i) {
        const KDL::Frame& frame = interpolatedPath[i].getFrame();
        double r, p, y;
        frame.M.GetRPY(r, p, y);

        geometry_msgs::PoseStamped pose;
        conversions::frameToPoseStampedRos(frame, pose);

        std::vector <geometry_msgs::Point> orientedFootprint;
        costmap->getOrientedFootprint(frame.p.x(), frame.p.y(), 0, orientedFootprint);
        double circumscribedRadius = costmap->getCircumscribedRadius();
        double inscribedRadius = costmap->getInscribedRadius();

        double c = collisionChecker.footprintCost(pose.pose.position,
                orientedFootprint,
                inscribedRadius,
                circumscribedRadius);

        if (c < 0) {
            ROS_INFO("Collision at pose: %f, %f", pose.pose.position.x, pose.pose.position.y);
            collision = true;
            break;
        }

    }

#ifdef DEBUG  
    stopwatch.stop();
    LOG("Collision checking using costmap2d_ros:");
    LOG("  - collision: %d", collision);
    LOG("  - step size: %f", interpolationStep);
    LOG("  - iterations : %lu", i);
    LOG("  - duration : %f ms", stopwatch.getElapsedTime());
#endif

    return collision;
}