#ifndef RANGER_H
#define RANGER_H

#include <string>
#include <memory>

#include <stage.hh>
#include <kdl/tree.hpp>
#include <ros/ros.h>

namespace amr
{

namespace stage
{

/** Abstract class which encapsulates Stg::ModelRanger.
  *
  * It is responsible for extracting the transform tree for the device and
  * publishing the current readings on proper topic(s). */
class Ranger
{

public:

  typedef std::shared_ptr<Ranger> Ptr;

  /** Factory method which creates the proper Ranger subclass (i.e. Sonar or
    * Laser) based on the properties of the @a model. */
  static Ranger::Ptr create(Stg::ModelRanger* model);

  /** Publish the current data supplied by the underlying Stage model. */
  virtual void publish(ros::Time timestamp) = 0;

  /** Build a tree of static transforms between the ranger device as a whole
    * and its components. */
  virtual void buildTransformTree(KDL::Tree& tree);

  /** Enable/disable the ranger device.
    *
    * Passing @c true means that the wrapper will subscribe to the underlying
    * Stage model, @c false means the opposite. */
  void enable(bool state);

  /** Get the name assigned to the ranger device. */
  const std::string& getName() const { return name_; };

  /** Get the current state (on/off) of the ranger device. */
  bool getState() const { return enabled_; };

  virtual ~Ranger();

protected:

  Ranger(Stg::ModelRanger* model, const std::string& name, const std::string& frame_id);

  /** Helper function which produces a segment of given size with proper link
    * and joint names. */
  KDL::Segment segmentFromPose(const Stg::Pose& pose, const std::string& name);

  /// Incapsulated Stage model.
  Stg::ModelRanger* model_;

  /// Unique name of the ranger device.
  std::string name_;

  /// Name of the coordinate frame that is associated with the ranger device.
  std::string frame_id_;

  /// Current state of the wrapper. True indicates that it is subscribed to
  /// the underlying Stage model.
  bool enabled_;

};

}

}

#endif /* RANGER_H */

