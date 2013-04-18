Summary
=======

ROS wrapper for the [Stage][] simulator. Requires that Stage 4.1 is installed in
*/opt/stage* or in the default path used by the ros-fuerte-stage package.

This package exposes only a subset of Stage's functionality via ROS.
Specifically, it supports only single-robot simulations. The Stage models that
are present in the simulated world are mapped to the corresponding ROS topics.

Its main purpose is to make the output of Stage more compatible with what Gazebo
produces and to be able to visualize the simulation also in RViz.

Nodes
=====

stage
-----

It is necessary to set the `/use_sim_time` parameter to __true__  *before*
starting the node. If this has not been done, the static transforms between
robot components will not be published.

### Subscribed topics

* `cmd_vel` (*geometry_msgs/Twist*)  
  velocity commands to control the robot

### Published topics

* `odom` (*nav_msgs/Odometry*)  
  odometry data from the robot

* `gt` (*nav_msgs/Odometry*)  
  ground truth position of the robot

* `scan_front` (*sensor_msgs/LaserScan*)  
  scans from the front laser

* `scan_rear` (*sensor_msgs/LaserScan*)  
  scans from the rear laser

* `sonar_pioneer` (*amr_msgs/Ranges*)  
  range readings from the pioneer sonar ring

* `sonar_braitenberg` (*amr_msgs/Ranges*)  
  range readings from the pair of braitenberg sonars

Note: laser and sonar topics are optional and depend on the availability of the
corresponding devices.

### Parameters

* `~world_file` (default: /opt/stage/share/stage/worlds/simple.world)  
  world description file

* `~window_width` (default: 400)  
  width of the Stage's GUI window

* `~window_height` (default: 300)  
  height of the Stage's GUI window

* `~headless` (default: false)  
  run Stage node without GUI window

* `~world_width`  
  width of the currently loaded world, set by the node during startup

* `~world_height`  
  height of the currently loaded world, set by the node during startup

### Dynamically reconfigurable parameters

Use the [rqt_reconfigure][] GUI to update these parameters in runtime:

* `~simulation_speed` (default: 10)  
  frequency of the simulation main loop (in Hz)

### Services provided

* `get_rangers` (*amr_srvs/GetRangers*)  
  returns the list of available ranger devices and their states (on/off)

* `switch_ranger` (*amr_srvs/SwitchRanger*)  
  turn on/off a particular ranger device

ranges_republisher
------------------

### Subscribed topics

* `ranges_in` (*amr_msgs/Ranges*)  
  input range readings, grouped in a single message

### Published topics

* `ranges_out` (*sensor_msgs/Range*)  
  output range readings, one reading per message

### Parameters

* `~filter_ids` (default: none)  
  comma-separated list of ids of the individual sonars that have to be
  republished; if not set then all readings are republished

[Stage]: http://playerstage.sourceforge.net/index.php?src=stage
[rqt_reconfigure]: http://ros.org/wiki/rqt_reconfigure
