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
#include <navigation_trajectory_msgs/Trajectory.h>

#include <string>
#include <vector>


class TrajectoryAdapterNode {
public:
    TrajectoryAdapterNode(std::string name);
    TrajectoryAdapterNode(const TrajectoryAdapterNode& orig);
    
    void setActualOdometry(const nav_msgs::Odometry& odometry);
    void setActualTrajectory(const navigation_trajectory_msgs::Trajectory& trajectory);
    void publishTrajectory(const navigation_trajectory_msgs::Trajectory& trajectory);
    void prune(const navigation_trajectory_msgs::Trajectory& globalTrajectory, navigation_trajectory_msgs::Trajectory& localTrajectory);
    bool slideWindow(const navigation_trajectory_msgs::Trajectory& globalTrajectory, 
        const nav_msgs::Odometry& actualOdometry,
        navigation_trajectory_msgs::Trajectory& localTrajectory);

     
    bool collisionCheck(const navigation_trajectory_msgs::Trajectory& trajectory, 
        const nav_msgs::Odometry& actualPose,
        nav_msgs::Odometry& newGoalPose);
    
    void replan(const navigation_trajectory_msgs::Trajectory& trajectory, 
        const nav_msgs::Odometry& actualPose, 
        const nav_msgs::Odometry& goalPose, 
        navigation_trajectory_msgs::Trajectory& newTrajectory);
    
    virtual ~TrajectoryAdapterNode();
    void controlLoop();
    
private:
    
    ros::NodeHandle node;
    
    ros::Subscriber trajectorySubscriber;
    ros::Subscriber odomSubscriber;
    ros::Publisher trajectoryPublisher;
    
    std::string nodeName;
    
    nav_msgs::Odometry actualOdometry;
    navigation_trajectory_msgs::Trajectory actualTrajectory;
    navigation_trajectory_msgs::Trajectory adaptedTrajectory;
    
};

#endif	/* TRAJECTORYFOLLOWERNODE_H */

