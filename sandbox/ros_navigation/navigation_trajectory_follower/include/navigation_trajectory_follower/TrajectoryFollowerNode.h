/* 
 * File:   TrajectoryPlannerNode.h
 * Author: alexey
 *
 * Created on May 30, 2012, 4:21 PM
 */

#ifndef TRAJECTORYFOLLOWERNODE_H
#define	TRAJECTORYFOLLOWERNODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <navigation_trajectory_planner/Trajectory.h>
#include <geometry_msgs/Twist.h>
#include <kdl/frames.hpp>

#include <string>
#include <vector>

class PositionController;


namespace KDL {
class Trajectory;
class Frame;
class Trajectory_Composite;
class VelocityProfile_Trap;

};

class TrajectoryFollowerNode {
public:
    TrajectoryFollowerNode(std::string name);
    TrajectoryFollowerNode(const TrajectoryFollowerNode& orig);
    void publishTwist(const geometry_msgs::Twist& twist);
   
    void setActualOdometry(const nav_msgs::Odometry& odometry);
    void setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory);
  
    
    virtual ~TrajectoryFollowerNode();
    void controlLoop();
    
private:
    
    double actualTime, startTime;
    
    KDL::Trajectory*createTrajectoryKDL(const navigation_trajectory_planner::Trajectory& trajectory);
    void createKDLFrame(const nav_msgs::Odometry& odometry, KDL::Frame& frame);
    
    KDL::Trajectory* actualTrajectoryKDL;
    
    ros::NodeHandle node;
    
    ros::Subscriber trajectorySubscriber;
    ros::Subscriber odomSubscriber;
    ros::Publisher twistPublisher;
    
    std::string nodeName;
    
    PositionController* controller;
    nav_msgs::Odometry actualOdometry;
    navigation_trajectory_planner::Trajectory actualTrajectory;
    geometry_msgs::Twist actualAcceleration;
    KDL::Twist desiredTwist;
    KDL::Frame desiredPose;
    
     KDL::Trajectory_Composite* trajectoryComposite_x;
     KDL::Trajectory_Composite* trajectoryComposite_y;
     KDL::VelocityProfile_Trap* velpref_x;
     KDL::VelocityProfile_Trap* velpref_y;
    
};

#endif	/* TRAJECTORYFOLLOWERNODE_H */

