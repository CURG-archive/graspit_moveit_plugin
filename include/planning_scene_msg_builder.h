#ifndef PLANNING_SCENE_MSG_BUILDER_H_
#define PLANNING_SCENE_MSG_BUILDER_H_

#include "moveit_msgs/PlanningScene.h"
#include "eigen3/Eigen/Geometry"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"

class Body;

class PlanningSceneMsgBuilder
{

private:

public:
  PlanningSceneMsgBuilder();

  //! Deletes the node handle and the db manager
  ~PlanningSceneMsgBuilder();

  void uploadPlanningSceneToMoveit();



private:

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  bool createCollisionMesh(Body *b, moveit_msgs::CollisionObject &collision_obj);

};

#endif //PLANNING_SCENE_MSG_BUILDER_H_
