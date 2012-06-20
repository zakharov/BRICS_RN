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

#ifndef CONVERSIONUTILS_H
#define	CONVERSIONUTILS_H

#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

#include <navigation_trajectory_planner/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <iostream>

class ConversionUtils {
public:

    void poseRosToKdl(const geometry_msgs::Pose& poseROS, KDL::Frame& poseKDL) {

        const geometry_msgs::Pose::_position_type& positionRef = poseROS.position;
        const geometry_msgs::Pose::_orientation_type& orientationRef = poseROS.orientation;
        KDL::Vector origin(positionRef.x, positionRef.y, positionRef.z);
        KDL::Rotation orientation(KDL::Rotation::Quaternion(orientationRef.x, orientationRef.y, orientationRef.z, orientationRef.w));

        poseKDL = KDL::Frame(orientation, origin);

    }

    void poseKdlToRos(const KDL::Frame& poseKDL, geometry_msgs::Pose& poseROS) {

        geometry_msgs::Pose::_position_type& position = poseROS.position;
        geometry_msgs::Pose::_orientation_type& orientation = poseROS.orientation;

        poseKDL.M.GetQuaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        position.x = poseKDL.p.x();
        position.y = poseKDL.p.y();
        position.z = poseKDL.p.z();

    }

    void twistRosToKdl(const geometry_msgs::Twist& twistROS, KDL::Twist& twistKDL) {

    }

    void twistKdlToRos(const KDL::Twist& twistKDL, geometry_msgs::Twist& twistROS) {
      //  std::cout <<  twistKDL << std::endl;
        twistROS.angular.x = twistKDL.rot.x();
        twistROS.angular.y = twistKDL.rot.y();
        twistROS.angular.z = twistKDL.rot.z();

        twistROS.linear.x = twistKDL.vel.x();
        twistROS.linear.y = twistKDL.vel.y();
        twistROS.linear.z = twistKDL.vel.z();
    }

    void pathRosToKdl(const std::vector<geometry_msgs::PoseStamped>& poseStampedArray, KDL::Path_Composite& path) {
        std::vector<geometry_msgs::PoseStamped>::const_iterator it;

        if (poseStampedArray.size() > 1) {

            geometry_msgs::PoseStamped p1 = poseStampedArray.front();
            geometry_msgs::PoseStamped p2;

            for (it = poseStampedArray.begin() + 1; it != poseStampedArray.end(); ++it) {
                p2 = *it;
                KDL::Frame f1, f2;
                poseRosToKdl(p1.pose, f1);
                poseRosToKdl(p2.pose, f2);

                KDL::Path_Line* pathLine = new KDL::Path_Line(f1, f2, new KDL::RotationalInterpolation_SingleAxis(), 0.01);
                path.Add(pathLine);

                p1 = p2;
            }

        }

    }

    void pathKdlToRos(const KDL::Path_Composite& path, std::vector<geometry_msgs::PoseStamped>& poseStampedArray) {


    }

    void trajectoryRosToKdl(const std::vector <nav_msgs::Odometry>& trajectoryROS, KDL::Trajectory& trajectroyKDL, double dt) {

    }

    void trajectoryKdlToRos(const KDL::Trajectory& trajectroyKDL, std::vector <nav_msgs::Odometry>& trajectoryROS, double dt) {
        double duration = trajectroyKDL.Duration();

        if (duration > 0.0 && dt > 0.0) {

            for (double time = 0; time <= duration; time += dt) {
                KDL::Frame poseKDL = trajectroyKDL.Pos(time);
                KDL::Twist twistKDL = trajectroyKDL.Vel(time);
            //    ROS_INFO("twist=%f",twistKDL);

                geometry_msgs::Pose poseROS;
                geometry_msgs::Twist twistROS;

                poseKdlToRos(poseKDL, poseROS);
                twistKdlToRos(twistKDL, twistROS);

                nav_msgs::Odometry odom;
                odom.pose.pose = poseROS;
                odom.twist.twist = twistROS;

                trajectoryROS.push_back(odom);
            }
        }
    }

};
#endif	/* CONVERSIONUTILS_H */

