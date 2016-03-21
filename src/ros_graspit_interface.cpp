/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "include/ros_graspit_interface.h"

#include <boost/foreach.hpp>
#include <cmath>
#include <Inventor/actions/SoGetBoundingBoxAction.h>

#include <src/DBase/DBPlanner/ros_database_manager.h>
#include <src/DBase/graspit_db_model.h>
#include <src/Collision/collisionStructures.h>

#include <include/mytools.h>
#include <include/world.h>
#include <include/body.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>
#include <include/scanSimulator.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>

#include <include/grasp.h>
#include <include/triangle.h>
#include <geometry_msgs/Point.h>

#include <thread>


namespace graspit_ros_planning
{


RosGraspitMoveitInterface::RosGraspitMoveitInterface() :
    root_nh_(NULL),
    priv_nh_(NULL)
{
}

RosGraspitMoveitInterface::~RosGraspitMoveitInterface()
{
  ROS_INFO("ROS GraspIt node stopping");
  ros::shutdown();
  delete root_nh_;
  delete priv_nh_;
}

//------------------------- Main class  -------------------------------

int RosGraspitMoveitInterface::init(int argc, char **argv)
{
  std::cout << "ThreadId" <<std::this_thread::get_id() << std::endl;
  //copy the arguments somewhere else so we can pass them to ROS
  int ros_argc = argc;
  char** ros_argv = new char*[argc];
  for (int i = 0; i < argc; i++)
  {
    ros_argv[i] = new char[strlen(argv[i])];
    strcpy(ros_argv[i], argv[i]);
  }
  //see if a node name was requested
  std::string node_name("ros_graspit_interface");
  for (int i = 0; i < argc - 1; i++)
  {
    //std::cerr << argv[i] << "\n";
    if (!strcmp(argv[i], "_name"))
    {
      node_name = argv[i + 1];
    }
  }
  //init ros
  ros::init(ros_argc, ros_argv, node_name.c_str());

  //init node handles
  root_nh_ = new ros::NodeHandle("");
  priv_nh_ = new ros::NodeHandle("~");

  mPlanningSceneBuilder = new PlanningSceneMsgBuilder();
  mPickupActionGoalBuilder = new GraspMsgPublisher(root_nh_);

  ROS_INFO("Using node name %s", node_name.c_str());
  for (int i = 0; i < argc; i++)
  {
    delete ros_argv[i];
  }
  delete ros_argv;

  ROS_INFO("MAKING SHAPE COMPLETION UI");

  QDialogButtonBox *controlBox = new QDialogButtonBox(Qt::Vertical);

  QPushButton * sendPlanningSceneButton = new QPushButton("send Scene", controlBox);
  sendPlanningSceneButton->setDefault(true);
  sendPlanningSceneButton->move(0,0);

  cb = new QComboBox(controlBox);
  cb->move(0,30);

  QPushButton * executeGraspButton = new QPushButton("execute Grasp", controlBox );
  executeGraspButton->setDefault(true);
  executeGraspButton->move(0,60);

  controlBox->resize(QSize(200,100));
  controlBox->show();

  QObject::connect(sendPlanningSceneButton, SIGNAL(clicked()), this, SLOT(onSendPlanningSceneButtonPressed()));
  QObject::connect(executeGraspButton, SIGNAL(clicked()), this, SLOT(onExecuteGraspButtonPressed()));

  QObject::connect(cb, SIGNAL(activated()), this, SLOT(fillComboBox()));

  ROS_INFO("ROS GraspIt node ready");
  return 0;
}

int RosGraspitMoveitInterface::mainLoop()
{
  ros::spinOnce();
  return 0;
}

void RosGraspitMoveitInterface::onSendPlanningSceneButtonPressed()
{

    mPlanningSceneBuilder->uploadPlanningSceneToMoveit();

    cb->clear();
    for(int i = 0; i < graspItGUI->getMainWorld()->getNumGB(); i++)
    {
        cb->addItem(graspItGUI->getMainWorld()->getGB(i)->getName());
    }
}

void RosGraspitMoveitInterface::onExecuteGraspButtonPressed()
{
    int gb_index = cb->currentIndex();
    mPickupActionGoalBuilder->sendPickupRequest(gb_index);
}



}
