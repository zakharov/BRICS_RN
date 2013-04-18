#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "amr_stage/laser.h"

amr::stage::Laser::Laser(Stg::ModelRanger* model, const std::string& name)
: Ranger(model, "scan_" + name, "/base_laser_" + name + "_link")
{
  ros::NodeHandle nh;
  publisher_ = nh.advertise<sensor_msgs::LaserScan>(name_, 10);
}

void amr::stage::Laser::publish(ros::Time timestamp)
{
  if (!enabled_) return;
  sensor_msgs::LaserScan msg;
  const Stg::ModelRanger::Sensor& sensor_ = model_->GetSensors()[0];
  if (sensor_.ranges.size())
  {
    msg.angle_min = - sensor_.fov / 2.0;
    msg.angle_max = + sensor_.fov / 2.0;
    msg.angle_increment = sensor_.fov / (double)(sensor_.sample_count - 1);
    msg.range_min = sensor_.range.min;
    msg.range_max = sensor_.range.max;
    msg.ranges.resize(sensor_.ranges.size());
    msg.intensities.resize(sensor_.intensities.size());
    for (size_t i = 0; i < sensor_.ranges.size(); i++)
    {
      msg.ranges[i] = sensor_.ranges[i];
      msg.intensities[i] = sensor_.intensities[i];
    }
    msg.header.frame_id = frame_id_;
    msg.header.stamp = timestamp;
    publisher_.publish(msg);
  }
}

