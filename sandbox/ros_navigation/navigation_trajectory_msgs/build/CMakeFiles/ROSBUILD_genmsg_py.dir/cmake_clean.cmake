FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/navigation_trajectory_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/navigation_trajectory_msgs/msg/__init__.py"
  "../src/navigation_trajectory_msgs/msg/_Trajectory.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
