#ifndef GRASP_MSG_PUBLISHER_H_
#define GRASP_MSG_PUBLISHER_H_

#include "moveit_msgs/PlanningScene.h"
#include "eigen3/Eigen/Geometry"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_interface/planning_interface.h"
#include <moveit/move_group_interface/move_group.h>

class Body;

class GraspMsgPublisher
{


public:
  GraspMsgPublisher(ros::NodeHandle *n);

  ~GraspMsgPublisher();

  void sendPickupRequest(int gb_index);



private:

  ros::Publisher grasp_pubisher;

};

#endif //GRASP_MSG_PUBLISHER_H_
