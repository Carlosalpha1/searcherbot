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

// MoveBaseActions.cpp

#include "searcherbot_examples/bt/MoveBaseActions.h"

namespace bt
{

// MoveBaseAction methods

  MoveBaseAction::MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
    : ActionNodeBase(name, config), init_tick_(false)
  {
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  BT::PortsList MoveBaseAction::providedPorts()
  {
    return { BT::InputPort<std::string>("duration") };
  }

  void MoveBaseAction::halt()
  {
    ROS_INFO("[%s] halt", this->name().c_str());
  }

  BT::NodeStatus MoveBaseAction::tick()
  {
    if (!init_tick_) {
      init_tick_ = true;
      init_time_= ros::Time::now();
      BT::Optional<double> msg = getInput<double>("duration");
      if (!msg)
      {
        throw BT::RuntimeError("error reading port [target]:", msg.error());
      }
      duration_ = msg.value();
    }

    if ((ros::Time::now() - init_time_).toSec() > duration_) {
      init_tick_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }


// ForwardAction methods
  ForwardAction::ForwardAction(const std::string& name, const BT::NodeConfiguration& config)
    : MoveBaseAction(name, config) {}

  void ForwardAction::halt()
  {
    MoveBaseAction::halt();
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    vel_pub.publish(msg);
  }

  BT::NodeStatus ForwardAction::tick()
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;
    msg.angular.z = 0;
    vel_pub.publish(msg);
    return BT::NodeStatus::FAILURE;
  }


// BackwardAction methods
  BackwardAction::BackwardAction(const std::string& name, const BT::NodeConfiguration& config)
    : MoveBaseAction(name, config) {}

  void BackwardAction::halt()
  {
    MoveBaseAction::halt();
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    vel_pub.publish(msg);
  }

  BT::NodeStatus BackwardAction::tick()
  {
    geometry_msgs::Twist msg;
    msg.linear.x = -0.5;
    msg.angular.z = 0;
    vel_pub.publish(msg);
    ROS_INFO("[%s] tick", this->name().c_str());
    return MoveBaseAction::tick();
  }


// TurnActions methods
  TurnAction::TurnAction(const std::string& name, const BT::NodeConfiguration& config)
    : MoveBaseAction(name, config) {}

  void TurnAction::halt()
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    vel_pub.publish(msg);
    ROS_INFO("[%s] halt", this->name().c_str());
  }

  BT::NodeStatus TurnAction::tick()
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0.5;
    vel_pub.publish(msg);
    ROS_INFO("[%s] tick", this->name().c_str());
    return MoveBaseAction::tick();
  }

} // namespace bt
