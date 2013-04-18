#ifndef LASER_H
#define LASER_H

#include "amr_stage/ranger.h"

namespace amr
{

namespace stage
{

/** Specialization of Ranger that works with Stage ranger models that are
  * actually lasers.
  *
  * Publishes sensor_msgs/LaserScan messages to the topic "/scan_XXX", where
  * "XXX" is the name passed to the constructor. The frame id of the messages
  * is "/base_laser_XXX_link". */
class Laser : public Ranger
{

public:

  Laser(Stg::ModelRanger* model, const std::string& name);

  virtual void publish(ros::Time timestamp);

private:

  ros::Publisher publisher_;

};

}

}

#endif /* LASER_H */

