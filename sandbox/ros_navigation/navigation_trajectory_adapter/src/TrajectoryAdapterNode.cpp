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

#include "navigation_trajectory_adapter/TrajectoryAdapterNode.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>
#include <navigation_trajectory_planner/TrajectoryPlanner.h>
#include <navigation_trajectory_planner/ConversionUtils.h>
#include <cmath>

#include <kdl/frames.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/trajectory_composite.hpp>

using namespace std;

TrajectoryAdapterNode* adapterNodeHandle = NULL;
costmap_2d::Costmap2DROS* costmap = NULL;
nav_core::BaseGlobalPlanner* planner;

void odomCallback(const nav_msgs::Odometry& odometry) {
    adapterNodeHandle->setActualOdometry(odometry);
}

void trajectoryCallback(const navigation_trajectory_msgs::Trajectory& trajectory) {
    adapterNodeHandle->setActualTrajectory(trajectory);
}

void TrajectoryAdapterNode::setActualOdometry(const nav_msgs::Odometry& odometry) {
    actualOdometry = odometry;
}

void TrajectoryAdapterNode::setActualTrajectory(const navigation_trajectory_msgs::Trajectory& trajectory) {
    globalTrajectory.trajectory.clear();
    //   prune(trajectory, adaptedTrajectory);
    globalTrajectory = trajectory;
}

void TrajectoryAdapterNode::publishTrajectory(const navigation_trajectory_msgs::Trajectory& trajectory) {
    trajectoryPublisher.publish(trajectory);
}

void TrajectoryAdapterNode::prune(const navigation_trajectory_msgs::Trajectory& globalTrajectory,
        navigation_trajectory_msgs::Trajectory& localTrajectory) {

    using namespace navigation_trajectory_msgs;

    Trajectory::_trajectory_type& localTrajectoryRef = localTrajectory.trajectory;
    const Trajectory::_trajectory_type& globalTrajectoryRef = globalTrajectory.trajectory;

    Trajectory::_trajectory_type::const_iterator it = globalTrajectoryRef.begin();


    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;
    geometry_msgs::Pose pose3;
    const float epsilon = 0.1;

    localTrajectoryRef.clear();
    localTrajectoryRef.push_back(*it);
    it++;

    pose2 = it->pose.pose;
    pose3 = (it + 1)->pose.pose;

    for (; it != globalTrajectoryRef.end(); it++) {

        pose1 = globalTrajectoryRef.back().pose.pose;
        pose2 = it->pose.pose;

        if ((it + 1) == globalTrajectoryRef.end()) {
            localTrajectoryRef.push_back(*(it));
            ROS_INFO("Adding last point");
            break;
        }

        pose3 = (it + 1)->pose.pose;

        float slope1 = (pose2.position.y - pose1.position.y) / (pose2.position.x - pose1.position.x);

        float slope2 = (pose3.position.y - pose1.position.y) / (pose3.position.x - pose1.position.x);

        float angle = atan(slope1);
        angle = angle + M_PI;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
        ROS_INFO("Angle: %f,  Quat: %f,  %f,  %f, %f", angle, quat.x, quat.y, quat.z, quat.w);

        if (fabs(slope1 - slope2) > epsilon) {

            localTrajectoryRef.push_back(*(it));
            localTrajectoryRef.back().pose.pose.orientation = globalTrajectoryRef.back().pose.pose.orientation;
            ROS_INFO("Adding a point");
        } else {

            ROS_INFO("Skipping a point");
        }


    }


}

TrajectoryAdapterNode::TrajectoryAdapterNode(std::string name) : nodeName(name) {

    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();

    trajectoryPublisher = globalNode.advertise<navigation_trajectory_msgs::Trajectory > ("localTrajectory", 1);
    odomSubscriber = globalNode.subscribe("odom", 1, &odomCallback);
    trajectorySubscriber = globalNode.subscribe("globalTrajectory", 1, &trajectoryCallback);
    
    trajectoryPlanner = new TrajectoryPlanner(planner,globalNode);
    ROS_INFO("trajectoryPlanner allocated");
    
    rollingWindowCursor = 0;
}

TrajectoryAdapterNode::~TrajectoryAdapterNode() {
    delete trajectoryPlanner;
}

double getDistance(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2) {
    double x1 = odom1.pose.pose.position.x;
    double y1 = odom1.pose.pose.position.y;
    double x2 = odom2.pose.pose.position.x;
    double y2 = odom2.pose.pose.position.y;

    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

bool TrajectoryAdapterNode::rollingWindow(const navigation_trajectory_msgs::Trajectory& globalTrajectory,
        const nav_msgs::Odometry& actualOdometry,
        navigation_trajectory_msgs::Trajectory& localTrajectory) {

    using namespace navigation_trajectory_msgs;

    
    int numberOfPoints = 20;
    
    Trajectory::_trajectory_type& localTrajectoryRef = localTrajectory.trajectory;
    const Trajectory::_trajectory_type& globalTrajectoryRef = globalTrajectory.trajectory;

    if (globalTrajectory.trajectory.empty()) {
        localTrajectoryRef.push_back(actualOdometry);
        return false;
    }
    
    if (globalTrajectory.trajectory.empty()) {
        localTrajectoryRef.push_back(actualOdometry);
        return false;
    }

    double min = getDistance(globalTrajectoryRef[0], actualOdometry);
    unsigned int minIndex = 0;

    for (unsigned int i = 1; i < globalTrajectoryRef.size(); i++) {
        double actual = getDistance(globalTrajectoryRef[i], actualOdometry);
        if (actual < min) {
            min = actual;
            minIndex = i;
        }
    }

    
    
    if (minIndex >= globalTrajectoryRef.size()) {
        localTrajectoryRef.push_back(globalTrajectoryRef.back());
        return false;
    }

    this->rollingWindowCursor = minIndex + 1;
    
    if (numberOfPoints > globalTrajectoryRef.size() - rollingWindowCursor)
        numberOfPoints = globalTrajectoryRef.size();
    else
        numberOfPoints += rollingWindowCursor;
    
    for (unsigned int i = minIndex + 1; i < numberOfPoints; i++) {
        localTrajectoryRef.push_back(globalTrajectoryRef[i]);
    }

    return true;
}

bool TrajectoryAdapterNode::collisionCheck(const navigation_trajectory_msgs::Trajectory& trajectory,
        const nav_msgs::Odometry& actualPose,
        nav_msgs::Odometry& newGoalPose) {

    costmap_2d::Costmap2D costmapCopy;
    costmap->clearRobotFootprint();
    costmap->getCostmapCopy(costmapCopy);

    base_local_planner::CostmapModel collisionChecker(costmapCopy);

    for (int i = 0; i < trajectory.trajectory.size(); i++) {
        nav_msgs::Odometry odom = trajectory.trajectory[i];
        geometry_msgs::Pose pose = odom.pose.pose;

        /**
         * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
         * @param  position The position of the robot in world coordinates
         * @param  footprint The specification of the footprint of the robot in world coordinates
         * @param  inscribed_radius The radius of the inscribed circle of the robot
         * @param  circumscribed_radius The radius of the circumscribed circle of the robot
         * @return Positive if all the points lie outside the footprint, negative otherwise
         */

        geometry_msgs::Point position = pose.position;


        vector <geometry_msgs::Point> orientedFootprint;
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


    return true;
}

void TrajectoryAdapterNode::replan(const navigation_trajectory_msgs::Trajectory& trajectory,
        const nav_msgs::Odometry& actualPose,
        const nav_msgs::Odometry& goalPose,
        navigation_trajectory_msgs::Trajectory& newTrajectory) {

    geometry_msgs::PoseStamped start;
    start.pose = actualPose.pose.pose;
    start.header.frame_id = "/odom";
    geometry_msgs::PoseStamped goal;
    goal.pose = goalPose.pose.pose;
    goal.header.frame_id = "/odom";
    // std::vector<geometry_msgs::PoseStamped> plan;
    //   planner->makePlan(start, goal, plan);

    
    ConversionUtils convert;
    KDL::Frame startKDL;
    convert.poseRosToKdl(start.pose, startKDL);
    KDL::Frame goalKDL;
    convert.poseRosToKdl(goal.pose, goalKDL);
    KDL::Path_Composite pathComposite;
    ROS_INFO("About to compute the path");
    trajectoryPlanner->setPathFrameId("/odom");
    ROS_INFO("Set PathFrameId");
    trajectoryPlanner->computePath(startKDL, goalKDL, pathComposite);
    ROS_INFO ("Recomputing that path");
    KDL::Trajectory_Composite trajectoryComposite;
    trajectoryPlanner->computeTrajectory(pathComposite, trajectoryComposite);
    ROS_INFO ("Converting path to a trajectory");
    convert.trajectoryKdlToRos(trajectoryComposite, newTrajectory, 0.2);


}

// Return RC low-pass filter output samples, given input samples,
 // time interval dt, and time constant RC
 double lowpass(double x, double last_y, double dt, double RC) {
   double y = 0;
   double alfa = dt / (RC + dt);
   y = alfa * x + (1-alfa) * last_y;
   return y;
 }

void TrajectoryAdapterNode::updateTrajctory(navigation_trajectory_msgs::Trajectory& updatedTrajectory,
        int startIndex, 
        navigation_trajectory_msgs::Trajectory& globalTrajectory) {
    
    /*ROS_INFO("UpdatedTrajectory: %i, startIndex: %i, globalTrajectory: %i", 
            updatedTrajectory.trajectory.size(), 
            startIndex, 
            globalTrajectory.trajectory.size());*/
    
    nav_msgs::Odometry startOdom = globalTrajectory.trajectory[startIndex];
    geometry_msgs::Twist start_twist = startOdom.twist.twist;
    
    ROS_INFO("Initial twist: %f, %f, %f", 
            startOdom.twist.twist.linear.x,
            startOdom.twist.twist.linear.y,
            startOdom.twist.twist.angular.z);
    
    for (int i = 0; i < updatedTrajectory.trajectory.size(); ++i) {
        if (startIndex + i < globalTrajectory.trajectory.size()) {
            
            geometry_msgs::Twist newTwist = updatedTrajectory.trajectory[i].twist.twist;
            
            start_twist.linear.x = lowpass(newTwist.linear.x, start_twist.linear.x, 0.1, 1);
            updatedTrajectory.trajectory[i].twist.twist.linear.x = start_twist.linear.x;
            
            
            
            start_twist.linear.y = lowpass(newTwist.linear.y, start_twist.linear.y, 0.1, 1);
            updatedTrajectory.trajectory[i].twist.twist.linear.y = start_twist.linear.y;
            
            start_twist.angular.z = lowpass(newTwist.angular.z, start_twist.angular.z, 0.1, 1);
            updatedTrajectory.trajectory[i].twist.twist.angular.z = start_twist.angular.z;
            
            ROS_INFO("Updated twist: %f, %f, %f", 
            updatedTrajectory.trajectory[i].twist.twist.linear.x,
            updatedTrajectory.trajectory[i].twist.twist.linear.y,
            updatedTrajectory.trajectory[i].twist.twist.angular.z);
            
            globalTrajectory.trajectory[startIndex + i] = updatedTrajectory.trajectory[i];
        }
    }
    
}

void TrajectoryAdapterNode::controlLoop() {
    navigation_trajectory_msgs::Trajectory* trajectoryRef;
    navigation_trajectory_msgs::Trajectory rollingWindowTrajectory;
    navigation_trajectory_msgs::Trajectory updatedTrajectory;

    rollingWindow(globalTrajectory, actualOdometry, rollingWindowTrajectory);
    // ROS_INFO("AdaptedTrajectory trajectory with size %d", rollingWindowTrajectory.trajectory.size());



  /*  nav_msgs::Odometry newGoalPose;
    bool noCollision = collisionCheck(rollingWindowTrajectory, actualOdometry, newGoalPose);


    if (!noCollision) {
        ROS_INFO("Collision trajectory");
        newGoalPose = globalTrajectory.trajectory.back();
        replan(rollingWindowTrajectory, actualOdometry, newGoalPose, updatedTrajectory);
        updateTrajctory(updatedTrajectory, rollingWindowCursor, globalTrajectory);
        //globalTrajectory = updatedTrajectory;
    } 

    */
    
    trajectoryRef = &rollingWindowTrajectory;
    publishTrajectory(*trajectoryRef);
}

int main(int argc, char **argv) {


    std::string name = "trajectory_adapter";
    ros::init(argc, argv, name);

        // TODO:: read topic names and configuration parameters from yaml files
    // Instantiating a costmap
    tf::TransformListener tf;
    costmap_2d::Costmap2DROS localCostmap("local_costmap", tf);
    costmap = &localCostmap;

    // Reading configuration parameters
    ros::NodeHandle node = ros::NodeHandle("~/");
    string localTrajectoryPlanner;
    node.param("local_costmap/trajectory_planner", localTrajectoryPlanner, string("navfn/NavfnROS"));

    // Loading a planner plugin
    nav_core::BaseGlobalPlanner* pathPlanner;
    //TODO: read planners names from configuration file
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgpLoader("nav_core", "nav_core::BaseGlobalPlanner");
    pathPlanner = bgpLoader.createClassInstance(localTrajectoryPlanner);
    pathPlanner->initialize(bgpLoader.getName(localTrajectoryPlanner), &localCostmap);
    planner = pathPlanner;
    
    TrajectoryAdapterNode trajectoryAdapterNode(name);
    adapterNodeHandle = &trajectoryAdapterNode;

    ROS_INFO("Initialize costmap size: %d, %d", localCostmap.getSizeInCellsX(), localCostmap.getSizeInCellsY());

    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        trajectoryAdapterNode.controlLoop();
        r.sleep();
    }

    return 0;
}
