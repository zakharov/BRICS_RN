#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/console.h>

#include "amr_stage/laser.h"
#include "amr_stage/ranger.h"
#include "amr_stage/sonar.h"

amr::stage::Ranger::Ranger(Stg::ModelRanger* model, const std::string& name, const std::string& frame_id)
: model_(model)
, name_(name)
, frame_id_(frame_id)
, enabled_(false)
{
}

amr::stage::Ranger::~Ranger()
{
  enable(false);
}

amr::stage::Ranger::Ptr amr::stage::Ranger::create(Stg::ModelRanger* model)
{
  auto& sensors = model->GetSensors();
  if (sensors.size() == 1 && sensors[0].sample_count != 1)
  {
    // TODO: distinguish between front and rear lasers.
    return Ranger::Ptr(new Laser(model, "front"));
  }
  else
  {
    switch (sensors.size())
    {
      case 2: return Ranger::Ptr(new Sonar(model, "braitenberg"));
      case 16: return Ranger::Ptr(new Sonar(model, "pioneer"));
      default: return Ranger::Ptr(new Sonar(model, "unknown"));
    }
  }
}

KDL::Segment amr::stage::Ranger::segmentFromPose(const Stg::Pose& pose, const std::string& name)
{
  // The segment will be named "xxx_link", and the corresponding joint will be
  // named "xxx_joint".
  // The provided name could already contain the "_link" substring in its name.
  // In such situation the joint name should be constructed by replacing it.
  std::string link_name;
  std::string joint_name;
  if (boost::algorithm::ends_with(name, "_link"))
  {
    link_name = name;
    joint_name = boost::algorithm::replace_last_copy(name, "_link", "_joint");
  }
  else
  {
    link_name = name + "_link";
    joint_name = name + "_joint";
  }
  using namespace KDL;
  return Segment(link_name, Joint(joint_name, Joint::None), Frame(Rotation::RPY(0, 0, pose.a), Vector(pose.x, pose.y, pose.z)));
}

void amr::stage::Ranger::buildTransformTree(KDL::Tree& tree)
{
  // Default implementation only creates segment between the tree root and
  // frame_id_.
  tree.addSegment(segmentFromPose(model_->GetPose(), frame_id_), tree.getRootSegment()->first);
}

void amr::stage::Ranger::enable(bool state)
{
  if (enabled_ != state)
  {
    enabled_ = state;
    if (state)
    {
      model_->Subscribe();
      ROS_INFO("Enabling %s...", name_.c_str());
    }
    else
    {
      model_->Unsubscribe();
      ROS_INFO("Disabling %s...", name_.c_str());
    }
  }
}

