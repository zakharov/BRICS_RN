#include <ros/ros.h>
#include <amr_msgs/Ranges.h>

#include "amr_stage/sonar.h"

amr::stage::Sonar::Sonar(Stg::ModelRanger* model, const std::string& name)
: Ranger(model, "sonar_" + name, "sonar_" + name + "_link")
{
  ros::NodeHandle nh;
  publisher_ = nh.advertise<amr_msgs::Ranges>(name_, 10);
}

void amr::stage::Sonar::publish(ros::Time timestamp)
{
  if (!enabled_) return;
  amr_msgs::Ranges msg;
  auto& sensors = model_->GetSensors();
  for (size_t i = 0; i < sensors.size(); i++)
  {
    sensor_msgs::Range m;
    m.field_of_view = sensors[i].fov;
    m.min_range = sensors[i].range.min;
    m.max_range = sensors[i].range.max;
    m.range = sensors[i].ranges[0];
    m.header.frame_id = "/" + getSensorName(i) + "_link";
    m.header.stamp = timestamp;
    msg.ranges.push_back(m);
  }
  publisher_.publish(msg);
}

void amr::stage::Sonar::buildTransformTree(KDL::Tree& tree)
{
  Ranger::buildTransformTree(tree);
  auto& sensors = model_->GetSensors();
  for (size_t i = 0; i < sensors.size(); i++)
  {
    tree.addSegment(segmentFromPose(sensors[i].pose, getSensorName(i)), frame_id_);
  }
}

std::string amr::stage::Sonar::getSensorName(size_t index)
{
  return name_ + "_" + boost::lexical_cast<std::string>(index);
}

