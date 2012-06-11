#include "rviz/plugin/type_registry.h"

#include "navigation_trajectory_display.h"


using namespace rviz;

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<Traj>("Traj");
  

}

