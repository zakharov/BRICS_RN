/* 
 * File:   conversions.h
 * Author: alexey
 *
 * Created on August 30, 2012, 1:52 PM
 */

#ifndef CONVERSIONS_H
#define	CONVERSIONS_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "navigation_trajectory_adapter/FrameWithId.h"
#include "navigation_trajectory_adapter/TwistWithId.h"

namespace conversions {

    inline void odometryRosToFrame(const nav_msgs::Odometry& odometry, FrameWithId& pose) {
        const geometry_msgs::Quaternion& orientation = odometry.pose.pose.orientation;
        const geometry_msgs::Point& position = odometry.pose.pose.position;
        pose.id = odometry.header.frame_id;
        pose.M = KDL::Rotation::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        pose.p = KDL::Vector(position.x, position.y, position.z);
    }

    inline void odometryRosToTwist(const nav_msgs::Odometry& odometry, TwistWithId& twist) {
        const geometry_msgs::Vector3& angular = odometry.twist.twist.angular;
        const geometry_msgs::Vector3& linear = odometry.twist.twist.linear;
        twist.id = odometry.header.frame_id;
        twist.rot = KDL::Vector(angular.x, angular.y, angular.z);
        twist.vel = KDL::Vector(linear.x, linear.y, linear.z);
    }

    inline void poseRosToFrame(const geometry_msgs::Pose& poseRos, const std::string& id, FrameWithId& pose) {
        const geometry_msgs::Quaternion& orientation = poseRos.orientation;
        const geometry_msgs::Point& position = poseRos.position;
        pose.id = id;
        pose.M = KDL::Rotation::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        pose.p = KDL::Vector(position.x, position.y, position.z);
    }

    inline void poseStampedRosToFrame(const geometry_msgs::PoseStamped& poseStamped, FrameWithId& pose) {
        poseRosToFrame(poseStamped.pose, poseStamped.header.frame_id, pose);
    }

    inline void frameToPoseRos(const FrameWithId& pose, geometry_msgs::Pose& poseRos, std::string& id) {
        id = pose.id;
        pose.M.GetQuaternion(poseRos.orientation.x,
                poseRos.orientation.y,
                poseRos.orientation.z,
                poseRos.orientation.w);
        poseRos.position.x = pose.p.x();
        poseRos.position.y = pose.p.y();
        poseRos.position.z = pose.p.z();
    }

    inline void frameToPoseStampedRos(const FrameWithId& pose, geometry_msgs::PoseStamped& poseStamped) {
        frameToPoseRos(pose, poseStamped.pose, poseStamped.header.frame_id);
    }

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

    inline void pathToPathRos(const std::vector<FrameWithId>& path, nav_msgs::Path& pathRos) {
        if (!path.empty()) {
            std::vector<FrameWithId>::const_iterator it = path.begin();
            pathRos.header.frame_id = it->id;
            pathToPathRos(path, pathRos.poses);
        }
    }

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

    inline void pathRosToPath(const nav_msgs::Path& pathRos, std::vector<FrameWithId>& path) {
        pathRosToPath(pathRos.poses, path);
    }

}
#endif	/* CONVERSIONS_H */

