#include "include/grasp_msg_publisher.h"

#include "graspit_msgs/Grasp.h"
#include "graspit_source/include/graspitGUI.h"
#include "graspit_source/include/world.h"
#include "graspit_source/include/body.h"
#include "graspit_source/include/robot.h"
#include "graspit_source/include/EGPlanner/searchState.h"

GraspMsgPublisher::GraspMsgPublisher(ros::NodeHandle *n)
{
    grasp_pubisher = n->advertise<graspit_msgs::Grasp>("/graspit/grasps", 5);
}

void GraspMsgPublisher::sendPickupRequest(int gb_index)
{
     GraspPlanningState *gps = new GraspPlanningState(graspItGUI->getMainWorld()->getCurrentHand());
     GraspableBody *b = graspItGUI->getMainWorld()->getGB(gb_index);
     gps->setObject(b);

     graspit_msgs::Grasp grasp;
     grasp.object_name = b->getName().toStdString().c_str();

     grasp.epsilon_quality=0;
     grasp.volume_quality=0;
     grasp.grasp_id=1;


     double dof[gps->getHand()->getNumDOF()];
     gps->getHand()->getDOFVals(dof);
     for(int i = 0; i < gps->getHand()->getNumDOF(); ++i)
     {
        grasp.final_grasp_dof.push_back(dof[i]);
        grasp.pre_grasp_dof.push_back(dof[i]);
     }

     transf finalHandTransform = gps->readPosition()->getCoreTran();

     float tx = finalHandTransform.translation().x() / 1000;
     float ty = finalHandTransform.translation().y() / 1000;
     float tz = finalHandTransform.translation().z() / 1000;
     float rw = finalHandTransform.rotation().w;
     float rx = finalHandTransform.rotation().x;
     float ry = finalHandTransform.rotation().y;
     float rz = finalHandTransform.rotation().z;

     grasp.final_grasp_pose.position.x=tx ;
     grasp.final_grasp_pose.position.y=ty;
     grasp.final_grasp_pose.position.z=tz;
     grasp.final_grasp_pose.orientation.w=rw;
     grasp.final_grasp_pose.orientation.x=rx;
     grasp.final_grasp_pose.orientation.y=ry;
     grasp.final_grasp_pose.orientation.z=rz;

     double moveDist = -50.0;
     transf pregraspHandTransform = (translate_transf(vec3(0,0,moveDist) * gps->getHand()->getApproachTran()) * gps->readPosition()->getCoreTran());
     tx = pregraspHandTransform.translation().x() / 1000;
     ty = pregraspHandTransform.translation().y() / 1000;
     tz = pregraspHandTransform.translation().z() / 1000;
     rw = pregraspHandTransform.rotation().w;
     rx = pregraspHandTransform.rotation().x;
     ry = pregraspHandTransform.rotation().y;
     rz = pregraspHandTransform.rotation().z;

     grasp.pre_grasp_pose.position.x=tx;
     grasp.pre_grasp_pose.position.y=ty;
     grasp.pre_grasp_pose.position.z=tz;
     grasp.pre_grasp_pose.orientation.w=rw;
     grasp.pre_grasp_pose.orientation.x=rx;
     grasp.pre_grasp_pose.orientation.y=ry;
     grasp.pre_grasp_pose.orientation.z=rz;

     grasp_pubisher.publish(grasp);
 }
