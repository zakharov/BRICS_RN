/* 
 * File:   TrajectoryPlannerNode.cpp
 * Author: alexey
 * 
 * Created on May 30, 2012, 4:21 PM
 */

#include "navigation_trajectory_adapter/TrajectoryAdapterNode.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <cmath>

using namespace std;

TrajectoryAdapterNode* adapterNodeHandle = NULL;


void odomCallback(const nav_msgs::Odometry& odometry)
{
    adapterNodeHandle->setActualOdometry(odometry);
}

void trajectoryCallback(const navigation_trajectory_planner::Trajectory& trajectory)
{
    adapterNodeHandle->setActualTrajectory(trajectory);
}

void  TrajectoryAdapterNode::setActualOdometry(const nav_msgs::Odometry& odometry) {
    actualOdometry = odometry;
}

void  TrajectoryAdapterNode::setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory) {
    prune(trajectory, adaptedTrajectory);
    publishTrajectory(adaptedTrajectory);
}

void TrajectoryAdapterNode::publishTrajectory(const navigation_trajectory_planner::Trajectory& trajectory) {
    trajectoryPublisher.publish(trajectory);
}

void TrajectoryAdapterNode::prune(const navigation_trajectory_planner::Trajectory& globalTrajectory, 
        navigation_trajectory_planner::Trajectory& localTrajectory) {

    using namespace navigation_trajectory_planner;
    
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
    pose3 = (it+1)->pose.pose;
    
    for (; it != globalTrajectoryRef.end(); it++) {

        pose1 = globalTrajectoryRef.back().pose.pose;
        pose2 = it->pose.pose;

        if ((it+1) == globalTrajectoryRef.end()) {
            localTrajectoryRef.push_back(*(it));
            ROS_INFO("Adding last point");
            break;
        }

        pose3 = (it+1)->pose.pose;

        float slope1 = (pose2.position.y - pose1.position.y) / (pose2.position.x - pose1.position.x);
        
        float slope2 = (pose3.position.y - pose1.position.y) / (pose3.position.x - pose1.position.x);

        float angle = atan(slope1);
        angle = angle + M_PI;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
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
                
    trajectoryPublisher = globalNode.advertise<navigation_trajectory_planner::Trajectory> ("adapted_trajectory", 1);
    odomSubscriber = globalNode.subscribe("odom", 1, &odomCallback);
    trajectorySubscriber = globalNode.subscribe("trajectory", 1, &trajectoryCallback); 
}

TrajectoryAdapterNode::~TrajectoryAdapterNode() {
    
}

void TrajectoryAdapterNode::controlLoop() {
    
    
    
}

int main(int argc, char **argv) {

  
    std::string name = "trajectory_adapter";
    ros::init(argc, argv, name);
    
    TrajectoryAdapterNode trajectoryAdapterNode(name);
    adapterNodeHandle = &trajectoryAdapterNode;
    
    ros::Rate r(100); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        trajectoryAdapterNode.controlLoop();
        r.sleep();
    }
    
    return 0;
}
