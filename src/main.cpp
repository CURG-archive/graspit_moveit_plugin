#include "include/ros_graspit_interface.h"

extern "C" Plugin* createPlugin() {
   //ros::init(NULL, NULL,"graspit_shape_completion_node");
  return new graspit_ros_planning::RosGraspitInterface();
}

extern "C" std::string getType() {
  return "ros_graspit_interface";
}
