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
#include <kdl/path_line.hpp>
#include <kdl/path_point.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_composite.hpp>
#include "tf/transform_datatypes.h"

#include "navigation_trajectory_common/FrameWithId.h"
#include "navigation_trajectory_common/TwistWithId.h"
#include "navigation_trajectory_common/Pose2D.h"
#include "navigation_trajectory_common/Twist2D.h"
#include "navigation_trajectory_common/KdlStream.h"

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
     * @brief Conversion from geometry_msgs::Twist ROS message to Twist2D data type.
     * 
     * Convert the geometry_msgs::Twist into a Twist2D instance, returned by
     * assignment to the suppliedd @p twist2d parametrer.
     * 
     * @param[in] geometry_msgs::Twist - twist ROS message.
     * @param[out] TwistWithId - converted twist.
     */
    inline void twistRosToTwist2d(const geometry_msgs::Twist& twist, Twist2D& twist2d) {
        double x = twist.linear.x;
        double y = twist.linear.y;
        double theta = twist.angular.z;
        twist2d = Twist2D(x, y, theta);
    }
    
    /**
     * @brief Conversion from Twist2D data type to geometry_msgs::Twist ROS message.
     * 
     * Convert the Twist2D into a geometry_msgs::Twist instance.
     * 
     * @param[in] TwistWithId - twist object.
     * @param[out] geometry_msgs::Twist - converted twist ROS message.
     */
    inline void twist2dToTwistRos(const Twist2D& twist2d, geometry_msgs::Twist& twist) {
        twist.linear.x = twist2d.getX();
        twist.linear.y = twist2d.getY();
        twist.angular.z = twist2d.getTheta();
    }

    /**
     * @brief Conversion from geometry_msgs::Pose ROS message to KDL::Frame data type.
     * 
     * Convert the ROS pose in a KDL::Frame instance and return the KDL::Frame.
     * 
     * @param[in] geometry_msgs::Pose - pose ROS message.
     * @param[out] KDL::Frame - result of the conversion.
     */
    inline void poseRosToFrameKdl(const geometry_msgs::Pose& poseRos, KDL::Frame& frame) {
        const geometry_msgs::Quaternion& orientation = poseRos.orientation;
        const geometry_msgs::Point& position = poseRos.position;
        frame.M = KDL::Rotation::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        frame.p = KDL::Vector(position.x, position.y, position.z);
    }

    /**
     * @brief Conversion from geometry_msgs::Pose ROS message to FrameWithId data type.
     * 
     * Convert the ROS pose in a KDL::Frame instance and return the KDL::Frame
     * wrapped in side a FrameWithID instance.  Uses poseRosToFrameKdl().
     *
     * @param[in] geometry_msgs::Pose - pose ROS message.
     * @param[in] std::string - frame id.
     * @param[out] FrameWithId - result of the conversion.
     */
    inline void poseRosToFrame(const geometry_msgs::Pose& poseRos, const std::string& id, FrameWithId& pose) {
        pose.id = id;
        poseRosToFrameKdl(poseRos, pose.getFrame());
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
     * @brief Conversion from geometry_msgs::Pose ROS message to Pose2D data type.
     * 
     * Convert the ROS pose in a Pose2D instance and return the Pose2D by
     * assigning to the supplied @p pose2d. 
     * 
     * @param[in] geometry_msgs::Pose - pose ROS message.
     * @param[out] Pose2D - result of the conversion.
     */
    inline void poseRosToPose2d(const geometry_msgs::Pose& pose, Pose2D& pose2d) {
        double x = pose.position.x;
        double y = pose.position.y;
        double theta = tf::getYaw(pose.orientation);
        pose2d = Pose2D(x, y, theta);
    }

    /**
     * @brief Conversion from KDL::Frame data type to geometry_msgs::Pose ROS message.
     *
     * Convert the KDL::Frame instance to a ROS geometry_msgs::Pose instance.
     *
     * @param[in] KDL::Frame - input frame, i.e. location and orientation.
     * @param[out] geometry_msgs::Pose - converted pose to a ROS message .
     */
    inline void frameKdlToPoseRos(const KDL::Frame& frame, geometry_msgs::Pose& poseRos) {
        frame.M.GetQuaternion(poseRos.orientation.x,
                poseRos.orientation.y,
                poseRos.orientation.z,
                poseRos.orientation.w);
        poseRos.position.x = frame.p.x();
        poseRos.position.y = frame.p.y();
        poseRos.position.z = frame.p.z();
    }

    /**
     * @brief Conversion from FrameWithId data type to geometry_msgs::Pose ROS message.
     *
     * Convert the FrameWithID instance to a ROS geometry_msgs::Pose instance and
     * return the ID of the FrameWithId as a separate result parameter.  Uses
     * frameKdlToPoseRos().
     * 
     * @param[in] FrameWithId - input frame, including frame id, location and orientation.
     * @param[out] geometry_msgs::Pose - converted pose to a ROS message .
     * @param[out] FrameWithId - converted id.
     */
    inline void frameToPoseRos(const FrameWithId& pose, geometry_msgs::Pose& poseRos, std::string& id) {
        id = pose.id;
        frameKdlToPoseRos(pose.getFrame(), poseRos);
    }

    /**
     * @brief Conversion from FrameWithId data type to geometry_msgs::PoseStamped ROS message.
     *
     * Convert the FrameWithID instance to a ROS geometry_msgs::PoseStamped instance. 
     * The FrameWithId::id member is copied to geometry_msgs::PoseStamped::header::frame_id.
     * This uses frameToPoseRos().
     * 
     * @param[in] FrameWithId - input frame, including frame id, location and orientation.
     * @param[out] geometry_msgs::PoseStamped - converted pose and frame id to a ROS message.
     */
    inline void frameToPoseStampedRos(const FrameWithId& pose, geometry_msgs::PoseStamped& poseStamped) {
        frameToPoseRos(pose, poseStamped.pose, poseStamped.header.frame_id);
    }

    /**
     * @brief Conversion from Pose2D type to geometry_msgs::Pose ROS message.
     *
     * Convert the Pose2D instance to a ROS geometry_msgs::Pose instance.
     * 
     * @param[in] Pose2D - input pose, including location and orientation.
     * @param[out] geometry_msgs::Pose - converted pose to a ROS message .
     */
    inline void pose2dToPoseRos(const Pose2D& pose2d, geometry_msgs::Pose& pose) {
        pose.position.x = pose2d.getX();
        pose.position.y = pose2d.getY();
        tf::Quaternion quat;

        quat.setRPY(0, 0, pose2d.getTheta());
        tf::quaternionTFToMsg(quat, pose.orientation);
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

    /**
     * @brief Conversion of a Pose2D to a KDL::Frame.
     *
     * Convert the Pose2D into a KDL::Frame instance.  The generated KDL::Frame
     * will have its Z coordinate and its roll and pitch rotation set to zero.
     * 
     * @param[in] Pose2D - input pose.
     * @param[out] KDL::Frame - result of conversion.
     */
    inline void pose2dToFrameKdl(const Pose2D& pose2d, KDL::Frame& pose) {
        pose = KDL::Frame(KDL::Rotation::RPY(0, 0, pose2d.getTheta()), KDL::Vector(pose2d.getX(), pose2d.getY(), 0));
    }

    /**
     * @brief Conversion from Twist2D data type to KDL::Twist.
     * 
     * Convert the Twist2D into a KDL::Twist instance.  The generated KDL::Twist
     * will have its Z translation and its rotation around X and Y set to zero.
     * 
     * @param[in] TwistWithId - twist object.
     * @param[out] KDL::Twist - converted twist object.
     */
     inline void twist2dToTwistKdl(const Twist2D& twist2d, KDL::Twist& twist) {
        twist = KDL::Twist(KDL::Vector(twist2d.getX(), twist2d.getY(), 0), KDL::Vector(0, 0, twist2d.getTheta()));
    }

    /**
     * @brief Conversion of a nav_msgs::Path ROS message to KDL::Path_Composite KDL path data type.
     *
     * This function convertes an arbitrary ROS path (composed of poses, i.e.
     * positions with associated orientations) to a KDL path.  The ROS path can
     * be any mixture of transaltional, rotational or compbined translational-roatational
     * segments.  Path segments that rotate on the spot are supported.
     *
     * The parameter @p eqradius assigns rotational motions a "length".  To be able
     * to traverse the path, we need a way to parametrize the whole motion,
     * translational and rotational combined as a 3D curve in the space of
     * translation and orientation (x,y,theta), by a single monotonic parameter
     * @c s, the generalized curve length.  For purely translational motion,
     * this is just the euclidian length of the path segment.  For rotational
     * motion, the change in orientation is translated in a curve length by taking
     * the length of an arc spanning the angular change in orientation on a
     * circle of radius @p eqradius.  For a segment describing a simultaneous
     * translation and rotation, the longer of the pure translational curve
     * length and the rotational cureve length is taken, i.e. we do @em not
     * calculate the true path integral.
     *
     * Good values for @p eqradius depend on the intended use of the constructed
     * path and possible on the kinematic constraints of the vehicle later
     * executing the path.  As a rule of thumb:
     * - if you want to later (re)sample the path with an euclidian step size @c es
     * and an angular step size of @c as, set @p eqradius such that @c as given
     * an arc length of @c es: @c eq = @c es / @c as.
     * - similar, if you know the maximum rotational and translational velocities
     * of the vehicle, make the arc length at maximum turn rate @c tr in a time step
     * match the translation distance at maximum speed @c ms in a time step:
     * @c eq = @c ms / @c tr.
     * - If no better information is available, try 0.5, matching 0.1 meter with
     * roughly 10 degree.
     *
     * @param[in] nav_msgs::Path - input path.
     * @param[in] double eqradius - equivalence radius for rotational components, see details.
     * @param[out] KDL::Path_Composite - result of conversion.
     */
    inline void pathRosToPathKdl(const nav_msgs::Path& pathRos, double eqradius, KDL::Path_Composite& path) {

        if (pathRos.poses.size() > 1) {

            std::vector<geometry_msgs::PoseStamped>::const_iterator it = pathRos.poses.begin();
            geometry_msgs::PoseStamped p1 = *it;
            it++;
            geometry_msgs::PoseStamped p2;

            KDL::Frame f1Kdl;
            poseRosToFrameKdl(p1.pose, f1Kdl);

            KDL::Frame f2Kdl;
            for (/* nothing */ ; it != pathRos.poses.end(); it++) {
                p2 = *it;
                poseRosToFrameKdl(p2.pose, f2Kdl);

                KDL::Path_Line* pathLine = new KDL::Path_Line(f1Kdl, f2Kdl, new KDL::RotationalInterpolation_SingleAxis(), eqradius);
                path.Add(pathLine, true);

                p1 = p2;
                f1Kdl = f2Kdl;
            }

        } else if (pathRos.poses.size() == 1) {
            ROS_WARN("Adding a single point to KDL::Path_composite.  Expect problems later on.");
            std::vector<geometry_msgs::PoseStamped>::const_iterator it = pathRos.poses.begin();
            geometry_msgs::PoseStamped p1 = *it;
            KDL::Frame f1Kdl;
            path.Add(new KDL::Path_Point(f1Kdl), true);
        }
    }

    /**
     * @brief Conversion of a std::vector<FrameWithId> to KDL::Path_Composite KDL path data type.
     *
     * This function convertes an arbitrary path (composed of Frames, i.e.
     * positions with associated orientations) to a KDL path.  The path can
     * be any mixture of transaltional, rotational or compbined translational-roatational
     * segments.  Path segments that rotate on the spot are supported.
     *
     * The parameter @p eqradius assigns rotational motions a "length".  See
     * detailed description of pathRosToPathKdl() for in-depth discussion of
     * this parameter.
     *
     * @param[in] nav_msgs::Path - input path.
     * @param[in] double eqradius - equivalence radius for rotational components, see details.
     * @param[out] KDL::Path_Composite - result of conversion.
     */
    inline void pathToPathKdl(const std::vector<FrameWithId>& path, double eqradius, KDL::Path_Composite& pathKDL) {

        if (path.size() > 1) {

            std::vector<FrameWithId>::const_iterator it = path.begin();
            FrameWithId p1 = *it;
            it++;
            FrameWithId p2;

            for (/* nothing */ ; it != path.end(); it++) {
                p2 = *it;

                KDL::Path_Line* pathLine = new KDL::Path_Line(p1.getFrame(), p2.getFrame(), new KDL::RotationalInterpolation_SingleAxis(), eqradius);
                pathKDL.Add(pathLine, true);

                p1 = p2;
            }

        } else if (path.size() == 1) {
            ROS_WARN("Adding a single point to KDL::Path_composite.  Expect problems later on.");
            std::vector<FrameWithId>::const_iterator it = path.begin();
            FrameWithId p1 = *it;
            pathKDL.Add(new KDL::Path_Point(p1.getFrame()), true);
        }
    }

    /**
     * @brief Conversion of a KDL::Trajectory_Composite to a ROS message.
     *
     * This function convertes a KDL trajectory into a simple, string-based ROS
     * message to use with ROS node wrappers around BRICS_RN (or KDL) classes.
     *
     * @param[in] KDL::Trajectory_Composite - input trajectory.
     * @param[out] navigation_trajectory_common::KdlStream - result of conversion.
     */
    inline void trajectoryKdlToRosKdlStream(const KDL::Trajectory_Composite &trajectory, navigation_trajectory_common::KdlStream &trajectoryMsg) {
        std::stringstream buffer;
        trajectory.Write(buffer);
        trajectoryMsg.KDLstream = buffer.str();
    }

    /**
     * @brief Conversion of a ROS KdlStream message to a KDL::Trajectory_Composite.
     *
     * This function convertes a ROS KdlStream message to a KDL trajectory.  To
     * be use with ROS node wrappers around BRICS_RN (or KDL) classes.
     *
     * @param[in] navigation_trajectory_common::KdlStream - input trajectory message.
     * @param[out] KDL::Trajectory_Composite - result of conversion.
     */
    inline void rosKdlStreamToTrajectoryKdl(const navigation_trajectory_common::KdlStream &trajectoryMsg, KDL::Trajectory_Composite &trajectory) {

        std::stringstream buffer;
        buffer.str(trajectoryMsg.KDLstream);
        char tmpBuf[80];
        KDL::EatWord(buffer, "[", tmpBuf, 80);
        if (strcmp(tmpBuf, "COMPOSITE") != 0) {
            ROS_WARN("Failed to read trajectory message!");
            return;
        }
        KDL::Eat(buffer, "[");
        int c = buffer.peek();
        while (c != ']' && !buffer.eof()) {
            trajectory.Add(KDL::Trajectory::Read(buffer));
            c=buffer.peek();
        }
    }

}
#endif	/* CONVERSIONS_H */

