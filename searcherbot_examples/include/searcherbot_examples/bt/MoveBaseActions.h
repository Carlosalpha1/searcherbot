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

#ifndef SEARCHERBOT_EXAMPLES_BT_MOVEBASEACTIONS_H_
#define SEARCHERBOT_EXAMPLES_BT_MOVEBASEACTIONS_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt
{

class MoveBaseAction : public BT::ActionNodeBase
{
public:
  MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  virtual void halt() override;
  virtual BT::NodeStatus tick() override;

  ros::Publisher vel_pub;
private:
  ros::NodeHandle nh_;
  ros::Time init_time_;
  bool init_tick_;
  double duration_; // seconds
};


class ForwardAction : public MoveBaseAction
{
public:
  ForwardAction(const std::string& name, const BT::NodeConfiguration& config);
  void halt() override;
  BT::NodeStatus tick() override;
};


class BackwardAction : public MoveBaseAction
{
public:
  BackwardAction(const std::string& name, const BT::NodeConfiguration& config);
  void halt() override;
  BT::NodeStatus tick() override;
};


class TurnAction : public MoveBaseAction
{
public:
  TurnAction(const std::string& name, const BT::NodeConfiguration& config);
  void halt() override;
  BT::NodeStatus tick() override;
};

} // namespace bt
#endif // SEARCHERBOT_EXAMPLES_BT_MOVEBASEACTIONS_H_
