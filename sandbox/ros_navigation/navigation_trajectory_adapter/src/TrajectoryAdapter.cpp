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

#include "navigation_trajectory_adapter/TrajectoryAdapter.h"
#include "navigation_trajectory_adapter/TrajectoryAdapterObserver.h"
#include "navigation_trajectory_adapter/TrajectoryAdapterObserverDummy.h"
#include "navigation_trajectory_adapter/Logger.h"

TrajectoryAdapter::TrajectoryAdapter() {
    this->observer = new TrajectoryAdapterObserverDummy();
}

TrajectoryAdapter::TrajectoryAdapter(TrajectoryAdapterObserver* observer) {
    this->observer = observer;
}

void TrajectoryAdapter::planPath(const FrameWithId& goal, std::vector<FrameWithId>& path) {
    //pathPlanner->makePlan(goal, path);
    LOG("Planning a path");
}

void TrajectoryAdapter::simplifyPath(const std::vector<FrameWithId>& path, std::vector<FrameWithId>& simplifiedPath) {
    LOG("Simplifying the path");
}

void TrajectoryAdapter::calculateVelocityProfile(const std::vector<FrameWithId>& path, std::vector<TwistWithId>& velocityProfile) {
    LOG("Converting the path to a trajectory");
}

void TrajectoryAdapter::updateOdometry(const FrameWithId& pose, const TwistWithId& twist) {
    actualOdometry.pose = pose;
    actualOdometry.twist = twist;
    actualOdometry.updated = true;

//    LOG("Updating odometry");
}

TrajectoryAdapter::~TrajectoryAdapter() {
    delete this->observer;
}

