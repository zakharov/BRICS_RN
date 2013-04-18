Summary
=======

This folder holds a copy of the enhanced stage simulator and related software
used in the Autonomous Mobile Robots (AMR) course at Bonn-Rhein-Sieg University
(BRSU).

It is here to serve as a lightweight replacement for the Gazebo simulator
usually used in ROS.  Stage has the advantage of being less demanding regarding
hardware OpenGL support and it is generally easier to setup and run.

Copied and used here with permission.  For comments regarding any of the AMR
packages in this folder, please contact Dr. Björn Kahl <bjoern.kahl@h-brs.de>

### Packages included

* amr_stage
  The Stage robot simulator.

* amr_stage_worlds
  Some world files for stage, including map data for the standard map server in ROS.

* amr_msgs
  A cut-down version of the message definition package of AMR, here only the messages needed for amr_stage are kept.

* amr_srvs
  A cut-down version of the service definition package of AMR, here only the services needed for amr_stage are kept.

