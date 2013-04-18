# Desc: definitions for different sonars and sonar arrays
# Author: Sergey Alexandrov

# .......................... p2dx sonars ............................. #

#  Prototype for p2dx sonar sensors
define p2dxsonar sensor
(
  # Define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.05 0.01 ]
  # Define the range bounds [min max]
  range [0 5.0]
  # Define the angular field of view in degrees
  fov 15
  # Define the number of samples spread over the fov
  samples 1
  # Define the color that ranges are drawn in the gui
  color_rgba [ 0 0 1 0.2 ]
)

define p2dx_sonar ranger
(
  # 16 transducers spread about the robot as follows
  p2dxsonar( pose [ 0.075 0.130 0 90 ] )
  p2dxsonar( pose [ 0.115 0.115 0 50 ] )
  p2dxsonar( pose [ 0.150 0.080 0 30 ] )
  p2dxsonar( pose [ 0.170 0.025 0 10 ] )
  p2dxsonar( pose [ 0.170 -0.025 0 -10 ] )
  p2dxsonar( pose [ 0.150 -0.080 0 -30 ] )
  p2dxsonar( pose [ 0.115 -0.115 0 -50 ] )
  p2dxsonar( pose [ 0.075 -0.130 0 -90 ] )
  p2dxsonar( pose [ -0.155 -0.130 0 -90 ] )
  p2dxsonar( pose [ -0.195 -0.115 0 -130 ] )
  p2dxsonar( pose [ -0.230 -0.080 0 -150 ] )
  p2dxsonar( pose [ -0.250 -0.025 0 -170 ] )
  p2dxsonar( pose [ -0.250 0.025 0 170 ] )
  p2dxsonar( pose [ -0.230 0.080 0 150 ] )
  p2dxsonar( pose [ -0.195 0.115 0 130 ] )
  p2dxsonar( pose [ -0.155 0.130 0 90 ] )
)

# ....................... AmigoBot sonars ............................ #

define amigo_sonar ranger
(
  p2dxsonar ( pose [ 0.073 0.105 0 90 ] )
  p2dxsonar ( pose [ 0.130 0.078 0 41 ] )
  p2dxsonar ( pose [ 0.154 0.030 0 15 ] )
  p2dxsonar ( pose [ 0.154 -0.030 0 -15 ] )
  p2dxsonar ( pose [ 0.130 -0.078 0 -41 ] )
  p2dxsonar ( pose [ 0.073 -0.105 0 -90 ] )
  p2dxsonar ( pose [ -0.146 -0.060 0 -145 ] )
  p2dxsonar ( pose [ -0.146 0.060 0 145 ] )
)

# ...................... Braitenberg sonars .......................... #

#  Prototype for braitenberg sonar sensors
define braitenbergsonar sensor
(
  # Define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.05 0.01 ]
  # Define the range bounds [min max]
  range [0 4.0]
  # Define the angular field of view in degrees
  fov 10
  # Define the number of samples spread over the fov
  samples 1
  # Define the color that ranges are drawn in the gui
  color_rgba [ 0 1 0 0.2 ]
)

define braitenberg_sonar ranger
(
  # 2 transducers spread about the robot as follows
  braitenbergsonar( pose [ 0.190  0.150 0  15 ] )
  braitenbergsonar( pose [ 0.190 -0.150 0 -15 ] )
)
