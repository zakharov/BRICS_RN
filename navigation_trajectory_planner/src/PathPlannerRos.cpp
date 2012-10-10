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

#include "navigation_trajectory_common/FrameWithId.h"
#include "navigation_trajectory_common/Conversions.h"

#include "navigation_trajectory_planner/PathPlannerRos.h"
#include "navigation_trajectory_planner/Stopwatch.h"
#include "navigation_trajectory_planner/Logger.h"

#include <nav_core/base_global_planner.h>

PathPlannerRos::PathPlannerRos(nav_core::BaseGlobalPlanner* planner) : planner(planner) {
}

PathPlannerRos::PathPlannerRos(const PathPlannerRos& orig) : planner(planner) {
}

PathPlannerRos::~PathPlannerRos() {
}

bool PathPlannerRos::computePath(const FrameWithId& start, const FrameWithId& goal, std::vector <FrameWithId>& path) {

    if (!planner) {
        ROS_ERROR("Planner instance in not allocated");
        return false;
    }

    geometry_msgs::PoseStamped startRos, goalRos;
    conversions::frameToPoseStampedRos(start, startRos);
    conversions::frameToPoseStampedRos(goal, goalRos);
    std::vector<geometry_msgs::PoseStamped> pathRos;

#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif    

    bool result = planner->makePlan(startRos, goalRos, pathRos);

#ifdef DEBUG    
    stopwatch.stop();
    LOG("A* path planning:");
    LOG("  - start: %f,%f,%f", startRos.pose.position.x, startRos.pose.position.y, startRos.pose.position.z);
    LOG("  - goal: %f,%f,%f", goalRos.pose.position.x, goalRos.pose.position.y, goalRos.pose.position.z);
    LOG("  - output size: %lu", pathRos.size());
    LOG("  - duration : %f ms", stopwatch.getElapsedTime());
#endif 

    conversions::pathRosToPath(pathRos, path);
    return result;
}