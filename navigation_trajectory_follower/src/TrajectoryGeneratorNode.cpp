/******************************************************************************
 * Copyright (c) 2011,2013
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov
 * Bjoern Kahl
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

#include "trajectory_generator/OmniDriveTrajectoryGenerator.h"

#include "navigation_trajectory_common/Conversions.h"

#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "navigation_trajectory_common/KdlStream.h"

#include "kdl/trajectory_composite.hpp"


/**
 * @file TrajectoryGeneratorNode.cpp
 *
 * @brief Implementation of a ROS wrapper around ITrajectoryGenerator interface.
 *
 * This file contains a complete ROS node offering trajectory generation.  It
 * listens on a configurable topic, defautl "rn_path" for ROS path messages and
 * converts such a path into a KDL trajectory using the ITrajectoryGenerator
 * interface.  The algorithm used can be change in the main() function by
 * instantiating a different implementation of that interface.  The generated
 * trajectory is send to a configurable topic, by default rn_trajectory.
 *
 */


using namespace std;

ros::Publisher trajectoryPublisher;
ITrajectoryGenerator *generator;


void pathCallback(const nav_msgs::Path& path) {

    KDL::Path_Composite pathKDL;
    conversions::pathRosToPathKdl(path, 0.5, pathKDL);

    KDL::Trajectory_Composite trajectoryKDL;
    generator->computeTrajectroy(pathKDL, trajectoryKDL);

    navigation_trajectory_common::KdlStream trajectoryMsg;
    conversions::trajectoryKdlToRosKdlStream(trajectoryKDL, trajectoryMsg);
    trajectoryMsg.stamp = path.header.stamp;
    
    trajectoryPublisher.publish(trajectoryMsg);
}


int main(int argc, char **argv) {

    std::string name = "trajectory_generator";
    ros::init(argc, argv, name);

    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();

    string inputPathTopic;
    string outputTrajectoryTopic;

    node.param<string > ("inputPathTopic", inputPathTopic, "rn_path");
    node.param<string > ("outputTrajectoryTopic", outputTrajectoryTopic, "rn_trajectory");

    /* ToDo: Load actual generator using Plugin-Lib */
    
    /* *** Change here to use a different generator algorithm *** */
    ITrajectoryGenerator *generator = new OmniDriveTrajectoryGenerator;
    
    ros::Subscriber pathSubscriber;
    trajectoryPublisher = globalNode.advertise<navigation_trajectory_common::KdlStream > (outputTrajectoryTopic, 1);
    pathSubscriber = globalNode.subscribe(inputPathTopic, 1, &pathCallback);

    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    delete generator;

    return 0;
}
