#include "amr_stage/base.h"

amr::stage::Base::Base(Stg::ModelPosition* model)
: model_(model)
{
  ros::NodeHandle nh;
  odometry_publisher_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  groundtruth_publisher_ = nh.advertise<nav_msgs::Odometry>("gt", 10);
  model_->Subscribe();
}

amr::stage::Base::~Base()
{
  model_->Unsubscribe();
}

void amr::stage::Base::publish(ros::Time timestamp)
{
  std::lock_guard<std::mutex> guard(lock_);
  // Publish odometry data.
  {
    nav_msgs::Odometry msg = createMessage(model_->est_pose, model_->GetVelocity());
    msg.header.frame_id = "odom";
    msg.header.stamp = timestamp;
    msg.child_frame_id = "base_footprint";
    odometry_publisher_.publish(msg);
    // Also broadcast transform.
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    tf::Point pt(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0);
    tf::Transform transform(q, pt);
    tf_.sendTransform(tf::StampedTransform(transform, timestamp, "odom", "base_footprint"));
  }
  // Publish ground truth.
  {
    nav_msgs::Odometry msg = createMessage(model_->GetGlobalPose(), model_->GetGlobalVelocity());
    msg.header.frame_id = "gt";
    msg.header.stamp = timestamp;
    groundtruth_publisher_.publish(msg);
  }
}

void amr::stage::Base::setSpeed(double x, double y, double a)
{
  std::lock_guard<std::mutex> guard(lock_);
  model_->SetSpeed(x, y, a);
}

nav_msgs::Odometry amr::stage::Base::createMessage(const Stg::Pose& p, const Stg::Velocity& v)
{
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = p.x;
  msg.pose.pose.position.y = p.y;
  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(p.a);
  msg.twist.twist.linear.x = v.x;
  msg.twist.twist.linear.y = v.y;
  msg.twist.twist.angular.z = v.a;
  return msg;
}

