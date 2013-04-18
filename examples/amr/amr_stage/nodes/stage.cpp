#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <cstdio>
#include <sys/stat.h>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Twist.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <stage.hh>

#include "amr_stage/base.h"
#include "amr_stage/ranger.h"
#include "amr_stage/laser.h"
#include "amr_stage/sonar.h"
#include "amr_srvs/SwitchRanger.h"
#include "amr_srvs/GetRangers.h"
#include "amr_stage/StageConfig.h"

namespace amr
{

namespace stage
{

class StageNode
{

public:

  StageNode(int argc, char** argv)
  : wall_rate_(10.0)
  {
    // Read settings from the parameter server.
    ros::NodeHandle pn("~");
    std::string world_file;
    int window_width, window_height;
    bool headless;
    pn.param<std::string>("world_file", world_file, "/opt/stage/share/stage/worlds/simple.world");
    pn.param<int>("window_width", window_width, 400);
    pn.param<int>("window_height", window_height, 300);
    pn.param<bool>("headless", headless, false);
    if (!fileExists(world_file))
    {
      throw std::runtime_error("world file does not exist");
    }
    // Initialize stage and load the world.
    ROS_INFO("Initializing simulator.");
    ROS_INFO("Libstage version: %s.", Stg::Version());
    ROS_INFO("World description file: %s.", world_file.c_str());
    Stg::Init(&argc, &argv);
    if (headless)
    {
      ROS_INFO("Mode: headless.");
      world_ = std::unique_ptr<Stg::World>(new Stg::World("Stage (ROS)"));
    }
    else
    {
      world_ = std::unique_ptr<Stg::World>(new Stg::WorldGui(window_width, window_height, "Stage (ROS)"));
    }
    world_->Load(world_file);
    world_->AddUpdateCallback((Stg::world_callback_t)invokeUpdate, this);
    // Create wrappers around robot components and establish transform tree.
    initComponents();
    createTransformsPublisher();
    readWorldSize(world_file);
    // Prepare a topic to publish simulation clock.
    ros::NodeHandle nh;
    nh.setParam("/use_sim_time", true);
    nh.setParam("/world_width", world_width_);
    nh.setParam("/world_height", world_height_);
    clock_publisher_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
    // Create services and subscriber.
    switch_ranger_service_ = nh.advertiseService("switch_ranger", &StageNode::switchRangerCallback, this);
    get_rangers_service_ = nh.advertiseService("get_rangers", &StageNode::getRangersCallback, this);
    cmdvel_subscriber_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &StageNode::cmdvelCallback, this);
    reconfigure_server_.setCallback(boost::bind(&StageNode::reconfigureCallback, this, _1, _2));
  };

  virtual ~StageNode() { };

  /** Infinite application loop. */
  void run()
  {
    world_->Start();
    while(ros::ok() && !world_->UpdateAll())
    {
      ros::spinOnce();
      Fl::wait();
      wall_rate_.sleep();
    }
    world_->QuitAll();
    world_->Stop();
  }

  /** This is a proxy function which is registered as a callback for Stage
    * updates.
    *
    * Stage does not allow a member function to be registered as a callback,
    * hence this trick with a static function. */
  static bool invokeUpdate(Stg::World* world, StageNode* node)
  {
    node->update();
    return false;
  }

  /** This is a "real" callback for Stage update events which publishes the
    * data from robot's sensors along with supportive information. */
  void update()
  {
    // Get the current simulation time.
    simulation_time_.fromSec(world_->SimTimeNow() / 1e6);
    // We are not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if (simulation_time_.sec == 0 && simulation_time_.nsec == 0)
    {
      ROS_DEBUG("Skipping initial simulation step to avoid publishing clock == 0.");
      return;
    }
    // Publish the current simulation time.
    rosgraph_msgs::Clock msg;
    msg.clock = simulation_time_;
    clock_publisher_.publish(msg);
    // Publish odometry and ground truth localization data.
    base_->publish(simulation_time_);
    // Publish static trasforms between robot components.
    fixed_transforms_publisher_->publishFixedTransforms();
    // Publish the data from all ranger devices.
    for (const auto& ranger : rangers_)
      ranger->publish(simulation_time_);
  }

  void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    base_->setSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
  }

  bool switchRangerCallback(amr_srvs::SwitchRanger::Request& request, amr_srvs::SwitchRanger::Response& response)
  {
    for (const auto& ranger : rangers_)
    {
      if (ranger->getName() == request.name)
      {
        ranger->enable(request.state);
        return true;
      }
    }
    return false;
  }

  bool getRangersCallback(amr_srvs::GetRangers::Request& request, amr_srvs::GetRangers::Response& response)
  {
    for (const auto& ranger : rangers_)
    {
      response.names.push_back(ranger->getName());
      response.states.push_back(ranger->getState());
    }
    return true;
  }

  void reconfigureCallback(amr_stage::StageConfig &config, uint32_t level)
  {
    ROS_INFO("Simulation speed: %.1f Hz.", config.speed);
    wall_rate_ = ros::WallRate(config.speed);
  }

private:

  /** Create wrappers around different Stage's Models that are present in the
    * world. */
  void initComponents()
  {
    ROS_INFO("Initializing robot components:");
    for (const auto& model : world_->GetAllModels())
    {
      std::string type = model->GetModelType();
      if (type == "ranger")
      {
        rangers_.push_back(Ranger::create(dynamic_cast<Stg::ModelRanger*>(model)));
        ROS_INFO(" - ranger [%s]", rangers_[rangers_.size() - 1]->getName().c_str());
      }
      if (type == "position")
      {
        if (!base_)
        {
          base_ = Base::UPtr(new Base(dynamic_cast<Stg::ModelPosition*>(model)));
          ROS_INFO(" - base");
        }
        else
        {
          ROS_WARN(" - base [skipped]");
        }
      }
    }
    if (!base_)
      throw std::runtime_error("no base (position) components found");
  }

  void createTransformsPublisher()
  {
    KDL::Tree tree("base_footprint");
    tree.addSegment(KDL::Segment("base_link", KDL::Joint("base_joint", KDL::Joint::None), KDL::Frame(KDL::Vector(0,0,0.5))), "base_footprint");
    for (const auto& ranger : rangers_)
    {
      KDL::Tree subtree("base_link");
      ranger->buildTransformTree(subtree);
      tree.addTree(subtree, "base_link");
    }
    fixed_transforms_publisher_ = std::unique_ptr<robot_state_publisher::RobotStatePublisher>(new robot_state_publisher::RobotStatePublisher(tree));
  }

  /** Read world file and extract world dimensions.
    *
    * Assumes that world is described by "floorplan" model, and the very first
    * "size" definition inside the very first "floorplan" is the sought size. */
  void readWorldSize(const std::string& world_file)
  {
    std::ifstream stream(world_file);
    std::string line;
    float x, y, z;
    bool inside_floorplan = false;
    if (stream)
      while (getline(stream, line))
      {
        if (!line.size())
          continue;
        if (inside_floorplan)
          if (sscanf(line.c_str(), "  size [%f %f %f]", &x, &y, &z) == 3)
          {
            world_width_ = x;
            world_height_ = y;
            break;
          }
        if (line == "floorplan")
          inside_floorplan = true;
      }
    stream.close();
  }

  /** Check if the file with given name exists. */
  static bool fileExists(const std::string& filename)
  {
    struct stat s;
    return (stat(filename.c_str(), &s) == 0);
  }

  std::unique_ptr<Stg::World> world_;
  ros::Publisher clock_publisher_;
  ros::Time simulation_time_;
  ros::WallRate wall_rate_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> fixed_transforms_publisher_;

  ros::Subscriber cmdvel_subscriber_;
  dynamic_reconfigure::Server<amr_stage::StageConfig> reconfigure_server_;

  // Switch ranger service
  ros::ServiceServer switch_ranger_service_;
  ros::ServiceServer get_rangers_service_;

  // Robot components
  Base::UPtr base_;
  std::vector<Ranger::Ptr> rangers_;

  // World size (according to the world description file)
  double world_width_;
  double world_height_;

};

}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stage");
  try
  {
    amr::stage::StageNode stage(argc, argv);
    stage.run();
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("Failed to initialize Stage node (\"%s\"), shutting down...", e.what());
    return 1;
  }
  return 0;
}

