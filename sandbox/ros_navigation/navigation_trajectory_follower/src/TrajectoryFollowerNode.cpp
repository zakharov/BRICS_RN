/* 
 * File:   TrajectoryPlannerNode.cpp
 * Author: alexey
 * 
 * Created on May 30, 2012, 4:21 PM
 */

#include "navigation_trajectory_follower/TrajectoryFollowerNode.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "kdl/trajectory_composite.hpp"
#include "kdl/path_composite.hpp"
#include <kdl/utilities/utility.h>

#include "VelocityProfile_Trap.h"
#include "kdl/path_line.hpp"
#include "kdl/rotational_interpolation_sa.hpp"
#include "kdl/trajectory_segment.hpp"
#include "kdl/trajectory_composite.hpp"


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
    actualAcceleration.linear.x = odometry.twist.twist.linear.x - actualOdometry.twist.twist.linear.x;
    actualAcceleration.linear.y = odometry.twist.twist.linear.y - actualOdometry.twist.twist.linear.y;
    
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
        
      
        double aVelX = desiredTwist.vel(0);//actualOdometry.twist.twist.linear.x;
        double aVelY = desiredTwist.vel(1);//actualOdometry.twist.twist.linear.y;
        
       
        
    
        KDL::Path_Composite* pathComposite = new KDL::Path_Composite();
        KDL::Path_Composite* pathComposite1 = new KDL::Path_Composite();
        KDL::Frame temp, start, end;
        
        
        
        
        createKDLFrame(*it, temp);
        
        trajectoryComposite_x = new KDL::Trajectory_Composite();
        trajectoryComposite_y = new KDL::Trajectory_Composite();
        
        ++it;
        
        double dPosX = desiredPose.p(0);
        double dPosY = desiredPose.p(1);
   
        double dVelX = desiredTwist.vel(0);
        double dVelY = desiredTwist.vel(1);
        
        
        KDL::Frame  last;
        createKDLFrame(trajectory.trajectory.back(), last);
        
        double dX = (last.p(0) - desiredPose.p(0))/KDL::sign( dVelX);
        double dY = (last.p(1) - desiredPose.p(1))/KDL::sign( dVelY);
        
        ROS_INFO("twist.p(0):%f, twist.p(1):%f", dVelX , dVelY );
        ROS_INFO("last.p(0):%f, last.p(1):%f, desiredPose.p(0):%f, desiredPose.p(1):%f", last.p(0), last.p(1),desiredPose.p(0),desiredPose.p(1) );
        
        while(it != trajectory.trajectory.end()) {
            start = temp;
            createKDLFrame(*it, end);
            temp = end;
 
            KDL::Path_Line* pathLineSegment = new KDL::Path_Line(start, end, new KDL::RotationalInterpolation_SingleAxis(), 0.01);
            KDL::Path_Line* pathLineSegment1 = new KDL::Path_Line(start, end, new KDL::RotationalInterpolation_SingleAxis(), 0.01);
            pathComposite->Add(pathLineSegment);
            pathComposite1->Add(pathLineSegment1);
            ++it;
        }
        
        
        
        
        velpref_x =  new KDL::VelocityProfile_Trap(0.5, 0.1);
        velpref_y =  new KDL::VelocityProfile_Trap(0.5, 0.1);
       
                
        velpref_x->SetProfile(0,KDL::sign(dY)*KDL::sign(dX)* sqrt(aVelX*aVelX+aVelY*aVelY), pathComposite->PathLength(),0);
        velpref_y->SetProfile(0,KDL::sign(dY)* sqrt(aVelY*aVelY), pathComposite1->PathLength(),0);
        ROS_INFO("signX: %f aVelX: %f", KDL::sign(dX), sqrt(aVelX*aVelX) );
        ROS_INFO("signY: %f aVelY: %f", KDL::sign(dY), sqrt(aVelY*aVelY) );
               
        trajectoryComposite_x->Add(new KDL::Trajectory_Segment(pathComposite, velpref_x));
        trajectoryComposite_y->Add(new KDL::Trajectory_Segment(pathComposite1, velpref_y));
        
        
        return /*trajectoryComposite*/NULL;
    }
    
    return NULL;
}

void  TrajectoryFollowerNode::setActualTrajectory(const navigation_trajectory_planner::Trajectory& trajectory) {
    actualTrajectory.trajectory.clear();
    actualTrajectory = trajectory;
    
  // if (actualTrajectoryKDL != NULL)
   //     delete actualTrajectoryKDL;
    
    
    trajectoryComposite_x = NULL;
    trajectoryComposite_y = NULL;
    
    
    /*actualTrajectoryKDL = */createTrajectoryKDL(this->actualTrajectory);
    
    startTime = ros::Time::now().toSec(); 
}

void TrajectoryFollowerNode::publishTwist(const geometry_msgs::Twist& twist) {
    twistPublisher.publish(twist);
}

TrajectoryFollowerNode::TrajectoryFollowerNode(std::string name) : nodeName(name) {
    
    trajectoryComposite_x = NULL;
    trajectoryComposite_y = NULL;
    
    ros::NodeHandle node = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle();
    
    actualTrajectoryKDL = NULL;
            
    twistPublisher = globalNode.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    odomSubscriber = globalNode.subscribe("odom", 1, &odomCallback);
    trajectorySubscriber = globalNode.subscribe("trajectory", 1, &trajectoryCallback); 
    
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
   actualTime = ros::Time::now().toSec() - startTime; 
    
   if (trajectoryComposite_x != NULL && trajectoryComposite_x->Duration() > 0) { 
    
       KDL::Frame desiredPoseX = trajectoryComposite_x->Pos(actualTime);
       KDL::Frame desiredPoseY = trajectoryComposite_x->Pos(actualTime);
       
       KDL::Twist desiredTwistX = trajectoryComposite_x->Vel(actualTime);;
       KDL::Twist desiredTwistY = trajectoryComposite_x->Vel(actualTime);;
    
       
        
        //desiredPose = actualTrajectoryKDL->Pos(actualTime);
        //desiredTwist = actualTrajectoryKDL->Vel(actualTime);
        
        desiredPose.p.data[0] = desiredPoseX.p(0);
        desiredPose.p.data[1] = desiredPoseY.p(1);
        ROS_INFO("about to fail");
        desiredTwist.vel.data[0] = desiredTwistX.vel(0);
        
        desiredTwist.vel.data[1] = desiredTwistY.vel(1);
        ROS_INFO("desiredTwist.vel.data[0]=%f", desiredTwist.vel.data[0]);
       
      /*  KDL::Twist desiredAcc = actualTrajectoryKDL->Acc(actualTime);
        */
        double dPosX = desiredPose.p(0);
        double dPosY = desiredPose.p(1);
   
        double dVelX = desiredTwist.vel(0);
        double dVelY = desiredTwist.vel(1);
        
      //  double dAccX = desiredAcc.vel(0);
       // double dAccY = desiredAcc.vel(1);
   
        double aPosX = actualOdometry.pose.pose.position.x;
        double aPosY = actualOdometry.pose.pose.position.y;
   
        double aVelX = actualOdometry.twist.twist.linear.x;
        double aVelY = actualOdometry.twist.twist.linear.y;
        
  //      double aAccX = actualAcceleration.linear.x;
   //     double aAccY = actualAcceleration.linear.y;
   
        double positionXError = dPosX- aPosX;
        double positionYError = dPosY- aPosY;
        
        double velocityXError = dVelX- aVelX;
        double velocityYError = dVelY- aVelY;
        
    //    double accXError = dAccX- aAccX;
     //   double accYError = dAccY- aAccY;
        //double positionError = vectorLength(dPosX, dPosY, aPosX, aPosY);
        //double velocityError = vectorLength(dVelX, dVelY, aVelX, aVelY);

        //double gain = 1;
        //double error = gain*positionError + velocityError; 
        double gain1 = 1;
        double gain2 = 1;
        double errorX = gain1*positionXError + velocityXError;
        double errorY = gain2*positionYError + velocityYError;
        
        
            
        
      //  ROS_INFO("Desired acc: dAccX=%f, dAccY=%f; Actual Acc: aAccX=%f, aAccY=%f", dAccX, dAccY, aAccX, aAccY);
        
      //  ROS_INFO("Desired odom: dPosX=%f, dPosY=%f; Actual odom: aPosX=%f, aPosY=%f", dPosX, dPosY, aPosX, aPosY);
      //  ROS_INFO("Desired vel: dVelX=%f, dVelY=%f; Actual vel: aVelX=%f, aVelY=%f", dVelX, dVelY, aVelX, aVelY);
      //  ROS_INFO("Error = %f, positionError=%f, velocityError=%f", error, positionError, velocityError);
        
        geometry_msgs::Twist cmd_vel;
        
        cmd_vel.linear.x = errorX;//desiredTwist.vel(0);
        cmd_vel.linear.y = errorY;//desiredTwist.vel(1);
        cmd_vel.angular.z = 0;
        
        
        publishTwist(cmd_vel);
   
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
