# Desc: Hokuyo URG-04LX-UG01 laser
# Author: Sergey Alexandrov

define hokuyosensor sensor
(
  range [ 0.0 4.0 ]
  pose [ 0 0 0.1 0 ]
  fov 240
  samples 683
  color_rgba [1 0 0 0.15]
)

define hokuyocommon ranger
(
  color "gray"
  size [ 0.05 0.05 0.07 ]
)

define hokuyobase hokuyocommon
(
  block(
    points 4
    point[0] [ 0 0 ]
    point[1] [ 0 1 ]
    point[2] [ 1 1 ]
    point[3] [ 1 0 ]
    z [0 0.07]
  )
)

define hokuyolaser hokuyobase ( hokuyosensor() )
