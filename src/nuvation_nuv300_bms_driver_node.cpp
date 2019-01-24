/*
 * Copyright 2018 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <cob_msgs/PowerState.h>
#include <nuv_bms_driver/nuv_300_bms.h>



int main(int argc, char** argv)
{
  // Initialize ROS, specify name of node
  ros::init(argc, argv, "nuvation_nuv300_bms_driver_node");

  // Create a handle for this node, initialize node
  ros::NodeHandle nh;

  // Create and initialize an instance of the BMS driver
  nuv_bms_driver::Nuv300BMS bms();

  return (0);
}
