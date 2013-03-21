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

#ifndef CONVERSIONS_H
#define	CONVERSIONS_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <kdl/frames.hpp>
#include <kdl/path_composite.hpp>

#include "navigation_trajectory_common/FrameWithId.h"
#include "navigation_trajectory_common/TwistWithId.h"

/**
 * @brief Conversions between ROS, BRICS_RN and KDL data types. 
 */
namespace conversions {

    /**
     * @brief Conversion from nav_msgs::Odometry ROS message to FrameWithId data type (only pose is converted).
     * 
     * Convert the nav_msgs::Odometry::pose::pose member in a KDL::Frame instance
     * and return the KDL::Frame wrapped in side a FrameWithID instance.  The 
     * FrameWithId::id member is set to the nav_msgs::Odometry::header::frame_id value.
     * 
     * @param[in] nav_msgs::Odometry - odometry ROS message.
     * @param[out] FrameWithId - converted pose from the odometry message.
     */
    inline void odometryRosToFrame(const nav_msgs::Odometry& odometry, FrameWithId& pose) {
        const geometry_msgs::Quaternion& orientation = odometry.pose.pose.orientation;
        const geometry_msgs::Point& position = odometry.pose.pose.position;
        pose.id = odometry.header.frame_id;
        KDL::Frame& poseKDL = pose.getFrame();
        poseKDL.M = KDL::Rotation::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        poseKDL.p = KDL::Vector(position.x, position.y, position.z);
    }

    /**
     * @brief Conversion from nav_msgs::Odometry ROS message to TwistWithId data type (only twist is converted).
     * 
     * Convert the nav_msgs::Odometry::twist::twist member in two KDL::Twist instance
     * and return the KDL::Twist wrapped in side a TwistWithID instance.  The 
     * TwistWithId::id member is set to the nav_msgs::Odometry::header::frame_id value.
     * 
     * @param[in] nav_msgs::Odometry - odometry ROS message.
     * @param[out] TwistWithId - converted twist from the odometry message.
     */
     inline void odometryRosToTwist(const nav_msgs::Odometry& odometry, TwistWithId& twist) {
        const geometry_msgs::Vector3& angular = odometry.twist.twist.angular;
        const geometry_msgs::Vector3& linear = odometry.twist.twist.linear;
        twist.id = odometry.header.frame_id;
        twist.getTwist().rot = KDL::Vector(angular.x, angular.y, angular.z);
        twist.getTwist().vel = KDL::Vector(linear.x, linear.y, linear.z);
    }

    /**
     * @brief Conversion from geometry_msgs::Pose ROS message to FrameWithId data type.
     * 
     * Convert the ROS pose in a KDL::Frame instance and return the KDL::Frame 
     * wrapped in side a FrameWithID instance. 
     * 
     * @param[in] geometry_msgs::Pose - pose ROS message.
     * @param[in] std::string - frame id.
     * @param[out] FrameWithId - result of the conversion.
     */
    inline void poseRosToFrame(const geometry_msgs::Pose& poseRos, const std::string& id, FrameWithId& pose) {
        const geometry_msgs::Quaternion& orientation = poseRos.orientation;
        const geometry_msgs::Point& position = poseRos.position;
        pose.id = id;
        KDL::Frame& poseKDL = pose.getFrame();
        poseKDL.M = KDL::Rotation::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        poseKDL.p = KDL::Vector(position.x, position.y, position.z);
    }

    /**
     * @brief Conversion from geometry_msgs::PoseStamped ROS message to FrameWithId data type.
     * 
     * Convert the ROS pose in a KDL::Frame instance and return the KDL::Frame 
     * wrapped in side a FrameWithID instance. The FrameWithId::id member is set 
     * to the geometry_msgs::PoseStamped::header::frame_id value.
     * 
     * @param[in] geometry_msgs::PoseStamped - pose ROS message with frame id.
     * @param[out] FrameWithId - result of the conversion.
     */
    inline void poseStampedRosToFrame(const geometry_msgs::PoseStamped& poseStamped, FrameWithId& pose) {
        poseRosToFrame(poseStamped.pose, poseStamped.header.frame_id, pose);
    }

    /**
     * @brief Conversion from FrameWithId data type to geometry_msgs::Pose ROS message.
     *
     * Convert the FrameWithID instance to a ROS geometry_msgs::Pose instance and
     * return the ID of the FrameWithId as a separate result parameter.
     * 
     * @param[in] FrameWithId - input frame, including frame id, location and orientation.
     * @param[out] geometry_msgs::Pose - converted pose to a ROS message .
     * @param[out] FrameWithId - converted id.
     */
    inline void frameToPoseRos(const FrameWithId& pose, geometry_msgs::Pose& poseRos, std::string& id) {
        id = pose.id;
        const KDL::Frame& poseKDL = pose.getFrame();
        poseKDL.M.GetQuaternion(poseRos.orientation.x,
                poseRos.orientation.y,
                poseRos.orientation.z,
                poseRos.orientation.w);
        poseRos.position.x = poseKDL.p.x();
        poseRos.position.y = poseKDL.p.y();
        poseRos.position.z = poseKDL.p.z();
    }

    /**
     * @brief Conversion from FrameWithId data type to geometry_msgs::PoseStamped ROS message.
     *
     * Convert the FrameWithID instance to a ROS geometry_msgs::PoseStamped instance. 
     * The FrameWithId::id member is copied to geometry_msgs::PoseStamped::header::frame_id
     * 
     * @param[in] FrameWithId - input frame, including frame id, location and orientation.
     * @param[out] geometry_msgs::PoseStamped - converted pose and frame id to a ROS message.
     */
    inline void frameToPoseStampedRos(const FrameWithId& pose, geometry_msgs::PoseStamped& poseStamped) {
        frameToPoseRos(pose, poseStamped.pose, poseStamped.header.frame_id);
    }

    /**
     * @brief Conversion of a FrameWithId vector (a path) to vector of geometry_msgs::PoseStamped ROS messages.
     *
     * See frameToPoseStampedRos() for details of the conversion process.
     *
     * @param[in] std::vector<FrameWithId> - input path.
     * @param[out] std::vector<geometry_msgs::PoseStamped> - result of conversion.
     */
    inline void pathToPathRos(const std::vector<FrameWithId>& path, std::vector <geometry_msgs::PoseStamped>& pathRos) {
        if (!path.empty()) {
            std::vector<FrameWithId>::const_iterator it = path.begin();
            geometry_msgs::PoseStamped pose;
            while (it != path.end()) {
                frameToPoseStampedRos(*it, pose);
                pathRos.push_back(pose);
                ++it;
            }
        }
    }

    /**
     * @brief Conversion of a FrameWithId vector (a path) to nav_msgs::Path ROS message.
     * 
     * See frameToPoseStampedRos() for details of the conversion process.
     *
     * @param[in] std::vector<FrameWithId>& path - input path.
     * @param[out] nav_msgs::Path - result of conversion.
     */
    inline void pathToPathRos(const std::vector<FrameWithId>& path, nav_msgs::Path& pathRos) {
        if (!path.empty()) {
            std::vector<FrameWithId>::const_iterator it = path.begin();
            pathRos.header.frame_id = it->id;
            pathToPathRos(path, pathRos.poses);
        }
    }

    /**
     * @brief Conversion of a geometry_msgs::PoseStamped vector to FrameWithId vector (a path).
     * 
     * See poseStampedRosToFrame() for details of the conversion process.
     *
     * @param[in]  std::vector<geometry_msgs::PoseStamped> - input path.
     * @param[out] std::vector<FrameWithId> - result of conversion.
     */
    inline void pathRosToPath(const std::vector <geometry_msgs::PoseStamped>& pathRos, std::vector<FrameWithId>& path) {
        if (!pathRos.empty()) {
            std::vector <geometry_msgs::PoseStamped>::const_iterator it = pathRos.begin();
            FrameWithId pose;
            while (it != pathRos.end()) {
                poseStampedRosToFrame(*it, pose);
                path.push_back(pose);
                ++it;
            }
        }
    }

    /**
     * @brief Conversion of a nav_msgs::Path ROS message to FrameWithId vector (a path).
     * 
     * See poseStampedRosToFrame() for details of the conversion process.
     *
     * @param[in] nav_msgs::Path - input path.
     * @param[out] std::vector<FrameWithId> - result of conversion.
     */
    inline void pathRosToPath(const nav_msgs::Path& pathRos, std::vector<FrameWithId>& path) {
        pathRosToPath(pathRos.poses, path);
    }

/*
    / **
     * @brief Conversion of a nav_msgs::Path ROS message to KDL::Path_Composite KDL path data type.
     * @param[in] nav_msgs::Path - input path.
     * @param[out] KDL::Path_Composite - result of conversion.
     * /
    inline void pathRosToPath(const nav_msgs::Path& pathRos, KDL::Path_Composite& path) {


        if (pathRos.poses.size() > 1) {

            std::vector<geometry_msgs::PoseStamped>::const_iterator it;
            geometry_msgs::PoseStamped p1 = pathRos.poses.front();
            geometry_msgs::PoseStamped p2;

            for (it = pathRos.poses.begin() + 1; it != pathRos.poses.end(); ++it) {
                p2 = *it;
                FrameWithId f1, f2;
                poseStampedRosToFrame(p1, f1);
                poseStampedRosToFrame(p2, f2);

                KDL::Frame f1Kdl = f1.getFrame();
                KDL::Frame f2Kdl = f2.getFrame();

                KDL::Path_Line* pathLine = new KDL::Path_Line(f1Kdl, f2Kdl, new KDL::RotationalInterpolation_SingleAxis(), 0.0001);
                path.Add(pathLine);

                p1 = p2;
            }

        }
    }*/

}
#endif	/* CONVERSIONS_H */

