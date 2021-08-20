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

#ifndef SEARCHERBOT_NAVIGATION_NAVIGATOR_H_
#define SEARCHERBOT_NAVIGATION_NAVIGATOR_H_
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace navigation
{

class Navigator
{
public:
  Navigator();

  bool navigateTo(double x, double y);
  bool navigateTo(const move_base_msgs::MoveBaseGoal& goal);

private:
  MoveBaseClient ac_;
};

}; // namespace navigation

#endif // SEARCHERBOT_NAVIGATION_NAVIGATOR_H_
