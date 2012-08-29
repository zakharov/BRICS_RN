/* 
 * File:   TrajectoryAdapterNode.cpp
 * Author: alexey
 *
 * Created on August 29, 2012, 2:23 PM
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdl/trajectory_composite.hpp>

#include "navigation_trajectory_adapter/TrajectoryAdapter.h"
#include "navigation_trajectory_adapter/TrajectoryAdapterObserver.h"
#include "navigation_trajectory_adapter/FrameWithId.h"

using namespace std;

class TrajectoryAdapterObserverROS : public TrajectoryAdapterObserver {

    void trajectoryCallback(KDL::Trajectory_Composite& trajectory) {
        ROS_INFO("Got new trajectory");
    }
};

class TrajectoryAdapterNodeState {
public:

    enum State {
        IDLING, PLANNING, COLLISION_CHECKING
    };

    TrajectoryAdapterNodeState() {
        set(IDLING);
    }

    TrajectoryAdapterNodeState(const TrajectoryAdapterNodeState& orig) {
        set(orig.get());
    }

    void set(State state) {
        this->state = state;
    }

    State get() const {
        return state;
    }


private:
    State state;

};

TrajectoryAdapter* trajectoryAdapter;
TrajectoryAdapterNodeState actualState;

void odometryCallback(const nav_msgs::Odometry& odometry) {
    ROS_INFO("Got new odometry");
    KDL::FrameWithId pose;
    KDL::Twist twist;
    
    //conversions::poseRosToFrameKDL (odometry.pose.pose, pose);
    //conversions::poseRosToFrameKDL (odometry.twist, twist);
    
    trajectoryAdapter->updateOdometry();
}

void goalCallback(const geometry_msgs::PoseStamped& goal) {
    ROS_INFO("Got new goal");
    actualState.set(TrajectoryAdapterNodeState::PLANNING);
}

double controlLoop() {

    double defaultCycleFrequencyInHz = 10.0;

    switch (actualState.get()) {
        case TrajectoryAdapterNodeState::IDLING:
            ROS_INFO("State IDLING");
            defaultCycleFrequencyInHz = 1.0;
            break;
        case TrajectoryAdapterNodeState::PLANNING:
            ROS_INFO("State PLANNING");
            actualState.set(TrajectoryAdapterNodeState::COLLISION_CHECKING);
            defaultCycleFrequencyInHz = 1.0;
            break;
        case TrajectoryAdapterNodeState::COLLISION_CHECKING:
            ROS_INFO("State COLLISION_CHECKING");
            defaultCycleFrequencyInHz = 10.0;
            break;
        default:
            ROS_ERROR("Unknown state");
    }

    return defaultCycleFrequencyInHz;
}

int main(int argc, char** argv) {

    std::string name = "trajectory_adapter";
    ros::init(argc, argv, name);

    trajectoryAdapter = new TrajectoryAdapter();

    ros::NodeHandle localNode = ros::NodeHandle("~/");
    ros::NodeHandle globalNode = ros::NodeHandle("");

    ros::Subscriber goalSubscriber;
    goalSubscriber = globalNode.subscribe("goal", 1, &goalCallback);

    ros::Subscriber odometrySubscriber;
    odometrySubscriber = globalNode.subscribe("odom", 1, &odometryCallback);

    actualState.set(TrajectoryAdapterNodeState::IDLING);

    while (ros::ok()) {
        ros::spinOnce();
        double hz = controlLoop();
        ros::Rate(hz).sleep();
    }


    delete trajectoryAdapter;
    ROS_INFO("bye...");
    return 0;
}

