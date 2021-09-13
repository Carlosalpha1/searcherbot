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

// Bump And Go using Behavior Trees
// It is a test behavior

#include <iostream>
#include <iomanip>
#include <string>

#include "ros/ros.h"
#include "ros/package.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

#include "searcherbot_examples/bt/ScanAction.h"
#include "searcherbot_examples/bt/MoveBaseActions.h"

#ifndef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bump_and_go");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  bt::laser_config::set_range(250, 500);
  
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<bt::ScanAction>("CheckLaser");
  factory.registerNodeType<bt::ForwardAction>("Forward");
  factory.registerNodeType<bt::BackwardAction>("Backward");
  factory.registerNodeType<bt::TurnAction>("Turn");

  std::string pkgpath = ros::package::getPath("searcherbot_examples");
  std::string xml_file = pkgpath + "/bt_xml/bumpgo.xml";

  auto tree = factory.createTreeFromFile(xml_file);

  // This logger saves state changes on file
  BT::FileLogger logger_file(tree, "/tmp/bt_trace.fbl");

  // This logger stores the execution time of each node
  BT::MinitraceLogger logger_minitrace(tree, "/tmp/bt_trace.json");

#ifndef ZMQ_FOUND
  // This logger publish status changes using ZeroMQ. Used By Groot
  BT::PublisherZMQ publisher_zmq(tree);
#endif

  BT::NodeStatus status;
  while (ros::ok() && status != BT::NodeStatus::SUCCESS)
  {
    status = tree.rootNode()->executeTick();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
