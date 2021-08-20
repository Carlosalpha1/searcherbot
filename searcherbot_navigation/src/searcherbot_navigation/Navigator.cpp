//  Copyright 2021 Carlos Caminero (Carlosalpha1)

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "searcherbot_navigation/Navigator.h"

namespace navigation
{

Navigator::Navigator()
  : ac_("/move_base", true)
{
  ac_.waitForServer();
  ROS_INFO("Navigation Server Activated!");
}

bool Navigator::navigateTo(double x, double y)
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  return this->navigateTo(goal);
}

bool Navigator::navigateTo(const move_base_msgs::MoveBaseGoal& goal)
{
  ROS_INFO("Sending Goal to (x:%f, y:%f)",
    goal.target_pose.pose.position.x,
    goal.target_pose.pose.position.y);
  
  ac_.sendGoal(goal);

  ac_.waitForResult();

  return ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

}; // namespace navigation
