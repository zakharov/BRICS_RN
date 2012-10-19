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

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_loader.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <kdl/trajectory_composite.hpp>

#include "navigation_trajectory_common/FrameWithId.h"
#include "navigation_trajectory_common/TwistWithId.h"
#include "navigation_trajectory_common/Conversions.h"

#include "navigation_trajectory_planner/PathPlannerRos.h"
#include "navigation_trajectory_planner/DouglasPeuckerApproximation.h"
#include "navigation_trajectory_planner/ChaikinCurveApproximation.h"

#include "navigation_trajectory_planner/CollisionCheckingRos.h"

using namespace std;

class TrajectoryAdapterNodeState {
public:

    enum State {
        IDLING, PLANNING, COLLISION_CHECKING
    };

    TrajectoryAdapterNodeState() {
        set(IDLING);
    }

    TrajectoryAdapterNodeState(const TrajectoryAdapterNodeState& orig) {
        set(orig.get());
    }

    void set(State state) {
        this->state = state;
    }

    State get() const {
        return state;
    }


private:
    State state;

};

TrajectoryAdapterNodeState actualState;

ros::Publisher pathPublisher, simplifiedPathPublisher;
TwistWithId actualTwist;
FrameWithId actualPose, goalPose;
IPathPlanner* pathPlanner;
ICollisionChecking* collisionChecker;

void odometryCallback(const nav_msgs::Odometry& odometry) {
    //  ROS_INFO("Got new odometry");

    conversions::odometryRosToFrame(odometry, actualPose);
    conversions::odometryRosToTwist(odometry, actualTwist);
}

void goalCallback(const geometry_msgs::PoseStamped& goal) {
    ROS_INFO("Got new goal");

    conversions::poseStampedRosToFrame(goal, goalPose);


    actualState.set(TrajectoryAdapterNodeState::PLANNING);
}

std::vector <FrameWithId> path, simplifiedPath;

double controlLoop() {

    double defaultCycleFrequencyInHz = 10.0;
     
    switch (actualState.get()) {
        case TrajectoryAdapterNodeState::IDLING:
        {
         //   ROS_INFO("State IDLING");
            break;
        }
        case TrajectoryAdapterNodeState::PLANNING:
        {
            ROS_INFO("State PLANNING");

            path.clear();
            simplifiedPath.clear();
            
            DouglasPeuckerApproximation peucker;
            ChaikinCurveApproximation chaikin;

            pathPlanner->computePath(actualPose, goalPose, path);
            
            if (path.empty())
                ROS_WARN("Can't plan a path");
            
            peucker.approximate(path, simplifiedPath);
                       
           //  chaikin.approximate(simplifiedPath, simplifiedPath, 2);

            nav_msgs::Path pathRos,  simplifiedPathRos;
            conversions::pathToPathRos(path, pathRos);
            conversions::pathToPathRos(simplifiedPath, simplifiedPathRos);
#ifdef DEBUG
            pathPublisher.publish(pathRos);
            simplifiedPathPublisher.publish(simplifiedPathRos);
#endif            
            actualState.set(TrajectoryAdapterNodeState::COLLISION_CHECKING);

            break;
        }
        case TrajectoryAdapterNodeState::COLLISION_CHECKING:
        {
            bool collision = collisionChecker->check(simplifiedPath, actualPose);
            if (collision)
                actualState.set(TrajectoryAdapterNodeState::PLANNING);
            break;
        }
        default:
            ROS_ERROR("Unknown state");
    }

    return defaultCycleFrequencyInHz;
}

int main(int argc, char** argv) {

    std::string name = "trajectory_adapter";
    ros::init(argc, argv, name);

    ros::NodeHandle localNode = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle("");

    ros::Subscriber goalSubscriber;
    goalSubscriber = globalNode.subscribe("goal", 1, &goalCallback);

    ros::Subscriber odometrySubscriber;
    odometrySubscriber = globalNode.subscribe("odom", 1, &odometryCallback);

    tf::TransformListener tf;
    costmap_2d::Costmap2DROS globalCostmap("local_costmap", tf);
    ROS_INFO("Initialize costmap size: %d, %d", globalCostmap.getSizeInCellsX(), globalCostmap.getSizeInCellsY());

    collisionChecker = new CollisionCheckingRos(&globalCostmap);

    nav_core::BaseGlobalPlanner* plannerRos;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgpLoader("nav_core", "nav_core::BaseGlobalPlanner");

    string globalTrajectoryPlanner;
    localNode.param("local_costmap/trajectory_adapter", globalTrajectoryPlanner, string("navfn/NavfnROS"));
    plannerRos = bgpLoader.createClassInstance(globalTrajectoryPlanner);
    plannerRos->initialize(bgpLoader.getName(globalTrajectoryPlanner), &globalCostmap);
    pathPlanner = new PathPlannerRos(plannerRos);


#ifdef DEBUG
    pathPublisher = globalNode.advertise<nav_msgs::Path > ("path", 1);
    simplifiedPathPublisher = globalNode.advertise<nav_msgs::Path > ("simplified_path", 1);
#endif

    actualState.set(TrajectoryAdapterNodeState::IDLING);

    while (ros::ok()) {
        ros::spinOnce();
        double hz = controlLoop();
        ros::Rate(hz).sleep();
    }


    delete pathPlanner;
    delete collisionChecker;
    delete plannerRos;

    ROS_INFO("bye...");
    return 0;
}

