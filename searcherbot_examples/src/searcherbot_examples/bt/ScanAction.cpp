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

// ScanAction.cpp

#include "searcherbot_examples/bt/ScanAction.h"

namespace bt
{
  namespace laser_config
  {
    void set_range(const int min_sample, const int max_sample)
    {
      LASER_MIN_SAMPLE = min_sample;
      LASER_MAX_SAMPLE = max_sample;
    }
  };

  ScanAction::ScanAction(const std::string& name)
    : ActionNodeBase(name, {}), laser_average_(1.0)
  {
    laser_sub_ = nh_.subscribe("/scan", 1, &ScanAction::laserCallback, this);
    ROS_INFO("LASER_MIN_SAMPLE [%d]", laser_config::LASER_MIN_SAMPLE);
    ROS_INFO("LASER_MAX_SAMPLE [%d]", laser_config::LASER_MAX_SAMPLE);
  }

  void ScanAction::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    // This Callback reads the laser scan data where
    // his first right sample is in index 0

    // Frontal Range of Laser Scanner
    int range[2] = {laser_config::LASER_MIN_SAMPLE, laser_config::LASER_MAX_SAMPLE};

    // calculating the average of the frontal range
    double sum = 0, counter = 0;
    for (int i = range[0]; i < range[1]; i++)
    {
      if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
        sum += msg->ranges[i];
        counter++;
      }
    }
    laser_average_ = sum/counter;
  }

  void ScanAction::halt()
  {
    ROS_INFO("[%s] halt", this->name().c_str());
  }

  BT::NodeStatus ScanAction::tick()
  {
    std::cout << "Laser_average: " << laser_average_ << std::endl;
    return (laser_average_ >= 1.0) ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
} // namespace bt
