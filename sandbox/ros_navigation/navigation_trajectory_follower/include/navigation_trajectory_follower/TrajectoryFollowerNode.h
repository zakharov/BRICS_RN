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

#include <string>
#include <vector>

class PositionController;



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
    
    ros::NodeHandle node;
    
    ros::Subscriber trajectorySubscriber;
    ros::Subscriber odomSubscriber;
    ros::Publisher twistPublisher;
    
    std::string nodeName;
    
    PositionController* controller;
    nav_msgs::Odometry actualOdometry;
    navigation_trajectory_planner::Trajectory actualTrajectory;
    
};

#endif	/* TRAJECTORYFOLLOWERNODE_H */

