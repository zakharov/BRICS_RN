# Desc: youBot robot
# Author: Sergey Alexandrov

define youbot position
(
  color "orange"       # Default color
  drive "omni"         # Omnidirectional drive
  gui_nose 1           # Draw a nose on the robot so we can see which way it points
  obstacle_return 1    # Can hit things
  ranger_return 0.5    # Reflects sonar beams
  blob_return 1        # Seen by blobfinders
  fiducial_return 1    # Seen as "1" fiducial finders

  localization "gps"
  localization_origin [0 0 0 0] # Start odometry at (0, 0, 0)

  # Actual size
  size [0.58 0.38 0.14]

  # Center of rotation is aligned with its geometrical center
  # TODO: verify
  origin [0 0 0 0]

  # Estimated mass in kg
  mass 24.0

  # Simplified body shape:
  block(
    points 34
    point[0]  [  0.28  0.19 ]
    point[1]  [  0.23  0.19 ]
    point[2]  [  0.22  0.18 ]
    point[3]  [  0.22  0.16 ]
    point[4]  [ -0.22  0.17 ]
    point[5]  [ -0.22  0.18 ]
    point[6]  [ -0.23  0.19 ]
    point[7]  [ -0.28  0.19 ]
    point[8]  [ -0.29  0.18 ]
    point[9]  [ -0.29  0.13 ]
    point[10] [ -0.28  0.12 ]
    point[11] [ -0.27  0.12 ]
    point[12] [ -0.28  0.06 ]
    point[13] [ -0.28 -0.06 ]
    point[14] [ -0.27 -0.12 ]
    point[15] [ -0.28 -0.12 ]
    point[16] [ -0.29 -0.13 ]
    point[17] [ -0.29 -0.18 ]
    point[18] [ -0.28 -0.19 ]
    point[19] [ -0.23 -0.19 ]
    point[20] [ -0.22 -0.18 ]
    point[21] [ -0.22 -0.17 ]
    point[22] [  0.22 -0.16 ]
    point[23] [  0.22 -0.18 ]
    point[24] [  0.23 -0.19 ]
    point[25] [  0.28 -0.19 ]
    point[26] [  0.29 -0.18 ]
    point[27] [  0.29 -0.13 ]
    point[28] [  0.28 -0.12 ]
    point[29] [  0.26 -0.12 ]
    point[30] [  0.26  0.12 ]
    point[31] [  0.28  0.12 ]
    point[32] [  0.29  0.13 ]
    point[33] [  0.29  0.18 ]
    z [0 0.12]
  )
)
