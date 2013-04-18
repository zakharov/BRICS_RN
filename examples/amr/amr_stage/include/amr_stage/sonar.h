#ifndef SONAR_H
#define SONAR_H

#include "amr_stage/ranger.h"

namespace amr
{

namespace stage
{

/** Specialization of Ranger that works with Stage ranger models that are
  * actually sonars.
  *
  * Publishes amr_msgs/Ranges messages to the topic "/sonar_XXX", where "XXX"
  * is the name passed to the constructor. The frame id of the messages is
  * "/sonar_XXX_link". */
class Sonar : public Ranger
{

public:

  Sonar(Stg::ModelRanger* model, const std::string& name);

  /** Reimpelementation of the default version provided by the Ranger class.
    *
    * Creates segments between the "central" device frame and individual sonar
    * sensors. */
  virtual void buildTransformTree(KDL::Tree& tree);

  virtual void publish(ros::Time timestamp);

private:

  std::string getSensorName(size_t index);

  ros::Publisher publisher_;

};

}

}

#endif /* SONAR_H */

