#include "include/ros_graspit_interface.h"

extern "C" Plugin* createPlugin() {
    return new graspit_ros_planning::RosGraspitMoveitInterface();
}

extern "C" std::string getType() {
    return "graspit_moveit_plugin";
}
