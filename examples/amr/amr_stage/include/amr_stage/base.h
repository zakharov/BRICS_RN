#ifndef BASE_H
#define BASE_H

#include <mutex>

#include <stage.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace amr
{

namespace stage
{

/** Wrapper around Stage's position model, which takes care of publishing
  * odometric and ground truth data, and provides a method to control the
  * speed of the robot.
  *
  * Odometry is published to the "/odom" topic, and also broadcasted as a
  * transform between the frames "/odom" and "/base_footprint". The ground
  * truth about the robot's position is published to the "/gt" topic. */
class Base
{

public:

  typedef std::unique_ptr<Base> UPtr;

  Base(Stg::ModelPosition* model);

  virtual ~Base();

  void publish(ros::Time timestamp);

  void setSpeed(double x, double y, double a);

private:

  /** Helper function to compose an odometry message from Stage's pose and
    * velocity datatypes. */
  static nav_msgs::Odometry createMessage(const Stg::Pose& p, const Stg::Velocity& v);

  /// Incapsulated Stage position model.
  Stg::ModelPosition* model_;

  ros::Publisher odometry_publisher_;

  ros::Publisher groundtruth_publisher_;

  tf::TransformBroadcaster tf_;

  /// A mutex to lock access to position model which may be used from both
  /// message and update callbacks.
  /// This was migrated from stageros package.
  std::mutex lock_;

};

}

}

#endif /* BASE_H */

