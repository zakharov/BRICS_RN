FILE(REMOVE_RECURSE
  "../src/navigation_trajectory_planner/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Trajectory.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Trajectory.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)