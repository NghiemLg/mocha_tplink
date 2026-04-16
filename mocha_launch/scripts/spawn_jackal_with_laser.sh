#!/bin/bash
# Spawn jackal with 3D lidar enabled
export JACKAL_LASER_3D=1
exec roslaunch jackal_gazebo spawn_jackal.launch "$@"
