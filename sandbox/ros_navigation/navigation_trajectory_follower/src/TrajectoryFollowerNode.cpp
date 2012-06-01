/* 
 * File:   TrajectoryPlannerNode.cpp
 * Author: alexey
 * 
 * Created on May 30, 2012, 4:21 PM
 */

#include "navigation_trajectory_follower/TrajectoryFollowerNode.h"
#include "omnidrive_controller/OmniDrivePositionController.h"
#include "tf/transform_datatypes.h"


using namespace std;

TrajectoryFollowerNode* followerNodeHandle = NULL;


void odomCallback(const nav_msgs::Odometry& odometry)
{
    followerNodeHandle->setActualOdometry(odometry);
}

void trajectoryCallback(const navigation_trajectory_planner::Trajectory& trajectory)
{
    followerNodeHandle->setActualTrajectory(trajectory);
    ROS_INFO("Got new trajectory size: %d", trajectory.trajectory.size());
}

void  TrajectoryFollowerNode::setActualOdometry(const nav_msgs::Odometry& odometry) {
    actualOdometry = odometry;
}

void  TrajectoryFollowerNode::setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory) {
    this->actualTrajectory.trajectory.clear();
    this->actualTrajectory = trajectory;
}

void TrajectoryFollowerNode::publishTwist(const geometry_msgs::Twist& twist) {
    twistPublisher.publish(twist);
}

TrajectoryFollowerNode::TrajectoryFollowerNode(std::string name) : nodeName(name) {
    
    VelocityRamp ramp1, ramp2;
    controller = new OmniDrivePositionController(ramp1,ramp2, Odometry(Pose2D(0.01,0.01,0.01)));
    
    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();
    
            
    twistPublisher = globalNode.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    odomSubscriber = globalNode.subscribe("odom", 1, &odomCallback);
    trajectorySubscriber = globalNode.subscribe("adapted_trajectory", 1, &trajectoryCallback); 
    
}

double getYaw (const geometry_msgs::Quaternion& orientation) {
    tf::Quaternion btOrientation;
    tf::quaternionMsgToTF(orientation, btOrientation);
    btScalar yaw, pitch, roll;
    btMatrix3x3(btOrientation).getEulerYPR(yaw, pitch, roll);
    return yaw;
}

TrajectoryFollowerNode::~TrajectoryFollowerNode() {
    delete controller;
}

void TrajectoryFollowerNode::controlLoop() {
    double x = this->actualOdometry.pose.pose.position.x;
    double y = this->actualOdometry.pose.pose.position.y;
    geometry_msgs::Quaternion quat = this->actualOdometry.pose.pose.orientation;
    double phi = getYaw(quat);
    
    Odometry odom(Pose2D(x,y,phi));
    Odometry newOdom = controller->computeNewOdometry(odom);

    geometry_msgs::Twist twist;
    
    twist.linear.x = newOdom.getTwist2D().getX();
    twist.linear.y = newOdom.getTwist2D().getY();
    twist.angular.z = newOdom.getTwist2D().getTheta();
    
    publishTwist(twist);

  //  ROS_INFO("%f, %f, %f", twist.linear.x, twist.linear.y, twist.angular.z );

    if (controller->isTargetOdometryReached() && actualTrajectory.trajectory.size() > 0) {

       
        nav_msgs::Odometry odom = actualTrajectory.trajectory.front();
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        quat = odom.pose.pose.orientation;
        
        phi = getYaw(quat);
       
        Odometry target(Pose2D(x, y, phi));
        
        ROS_INFO("Setting a new goal x=%f y=%f phi=%f", x, y, phi);
        controller->setTargetOdometry(target);
        actualTrajectory.trajectory.erase(actualTrajectory.trajectory.begin());
    }
}

int main(int argc, char **argv) {

  
    std::string name = "trajectory_follower";
    ros::init(argc, argv, name);
//    tf::TransformListener tf;
    
    TrajectoryFollowerNode trajectoryFollowerNode(name);
    followerNodeHandle = &trajectoryFollowerNode;
    
    ros::Rate r(100); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        trajectoryFollowerNode.controlLoop();
        r.sleep();
    }
    
    return 0;
}
