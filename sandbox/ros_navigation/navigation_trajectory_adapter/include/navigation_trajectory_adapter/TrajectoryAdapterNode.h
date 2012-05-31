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

#include <string>
#include <vector>


class TrajectoryAdapterNode {
public:
    TrajectoryAdapterNode(std::string name);
    TrajectoryAdapterNode(const TrajectoryAdapterNode& orig);
    
    void setActualOdometry(const nav_msgs::Odometry& odometry);
    void setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory);
    void publishTrajectory(const navigation_trajectory_planner::Trajectory& trajectory);
    void prune(const navigation_trajectory_planner::Trajectory& globalTrajectory, navigation_trajectory_planner::Trajectory& localTrajectory);

     
    virtual ~TrajectoryAdapterNode();
    void controlLoop();
    
private:
    
    ros::NodeHandle node;
    
    ros::Subscriber trajectorySubscriber;
    ros::Subscriber odomSubscriber;
    ros::Publisher trajectoryPublisher;
    
    std::string nodeName;
    
    nav_msgs::Odometry actualOdometry;
    navigation_trajectory_planner::Trajectory actualTrajectory;
    navigation_trajectory_planner::Trajectory adaptedTrajectory;
    
};

#endif	/* TRAJECTORYFOLLOWERNODE_H */

