/* 
 * File:   TrajectoryPlannerNode.cpp
 * Author: alexey
 * 
 * Created on May 30, 2012, 4:21 PM
 */

#include "navigation_trajectory_follower/TrajectoryFollowerNode.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "kdl/trajectory_composite.hpp"
#include "kdl/path_composite.hpp"
#include "kdl/path_line.hpp"
#include "kdl/rotational_interpolation_sa.hpp"
#include "kdl/velocityprofile_trap.hpp"
#include "kdl/trajectory_segment.hpp"


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

void TrajectoryFollowerNode::createKDLFrame(const nav_msgs::Odometry& odometry, KDL::Frame& frame) {
    
    geometry_msgs::Pose pose = odometry.pose.pose;
    
    KDL::Rotation orientation(KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    KDL::Vector origin(pose.position.x, pose.position.y, pose.position.z);
    
    frame = KDL::Frame(orientation, origin);
}

KDL::Trajectory* TrajectoryFollowerNode::createTrajectoryKDL(const navigation_trajectory_planner::Trajectory& trajectory) {
    
    using namespace navigation_trajectory_planner;
    
    Trajectory::_trajectory_type::const_iterator it = trajectory.trajectory.begin();
    
    if (it != trajectory.trajectory.end()) {
    
        KDL::Path_Composite* pathComposite = new KDL::Path_Composite();
        KDL::Frame temp, start, end;
        createKDLFrame(*it, temp);
        ++it;
        
        while(it != trajectory.trajectory.end()) {
            start = temp;
            createKDLFrame(*it, end);
            temp = end;
 
            KDL::Path_Line* pathLineSegment = new KDL::Path_Line(start, end, new KDL::RotationalInterpolation_SingleAxis(), 0.01);
            pathComposite->Add(pathLineSegment);
            
            ++it;
        }
        
        KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.5, 0.05);
        velpref->SetProfile(0, pathComposite->PathLength());
        return new KDL::Trajectory_Segment(pathComposite, velpref);
    }
    
    return NULL;
}

void  TrajectoryFollowerNode::setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory) {
    actualTrajectory.trajectory.clear();
    actualTrajectory = trajectory;
    
    if (actualTrajectoryKDL != NULL)
        delete actualTrajectoryKDL;
    
    actualTrajectoryKDL = createTrajectoryKDL(this->actualTrajectory);
}

void TrajectoryFollowerNode::publishTwist(const geometry_msgs::Twist& twist) {
    twistPublisher.publish(twist);
}

TrajectoryFollowerNode::TrajectoryFollowerNode(std::string name) : nodeName(name) {
    
    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();
    
    actualTrajectoryKDL = NULL;
            
    twistPublisher = globalNode.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    odomSubscriber = globalNode.subscribe("odom", 1, &odomCallback);
    trajectorySubscriber = globalNode.subscribe("adapted_trajectory", 1, &trajectoryCallback); 
    
}

TrajectoryFollowerNode::~TrajectoryFollowerNode() {
   
}

double vectorLength(double x1, double x2, double y1, double y2) {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void TrajectoryFollowerNode::controlLoop() {
   
   // double x = this->actualOdometry.pose.pose.position.x;
   // double y = this->actualOdometry.pose.pose.position.y;
   // geometry_msgs::Quaternion quat = this->actualOdometry.pose.pose.orientation;
   KDL::Frame desiredPose = actualTrajectoryKDL->Pos(actualTime);
   KDL::Twist desiredTwist = actualTrajectoryKDL->Pos(actualTime);
   
   double dPosX = desiredPose.p(0);
   double dPosY = desiredPose.p(1);
   
   double dVelX = desiredTwist.vel(0);
   double dVelY = desiredTwist.vel(1);
   
   double aPosX = actualOdometry.pose.pose.position.x;
   double aPosY = actualOdometry.pose.pose.position.y;
   
   double aVelX = actualOdometry.twist.twist.linear.x;
   double aVelY = actualOdometry.twist.twist.linear.y;
   
   double positionError = vectorLength(dPosX, dPosY, aPosX, aPosY);
   double velocityError = vectorLength(dVelX, dVelY, aVelX, aVelY);

   double gain = 1;
   double error = gain*positionError + velocityError; 
   
   ROS_INFO("Desired odom: dPosX=%f, dPosY=%f; Actual odom: aPosX=%f, aPosY=%f", dPosX, dPosY, aPosX, aPosY);
   ROS_INFO("Desired vel: dVelX=%f, dVelY=%f; Actual vel: aVelX=%f, aVelY=%f", dVelX, dVelY, aVelX, aVelY);
   ROS_INFO("Error = %f, positionError=%f, velocityError=%f", error, positionError, velocityError);
    
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
