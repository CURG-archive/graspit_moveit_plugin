#!/bin/bash

export GRASPIT=$(rospack find graspit)/graspit_source
export GRASPIT_PLUGIN_DIR=$(dirname $(catkin_find libros_graspit_interface.so))

rosrun graspit graspit -p libros_graspit_interface
