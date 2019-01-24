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
#include <modbus/modbus.h>


namespace nuv_bms_driver
{

class Nuv300BMS
{
public:
  Nuv300BMS();
  ~Nuv300BMS();

protected:
  void timer_callback(const ros::TimerEvent& event);
  ros::NodeHandle node_handle_;
  ros::Timer timer_;

  std::string bms_ip_address_;  // ip address of the BMS
  int bms_port_;                // network port to BMS

  ros::Publisher power_state_pub_;
  unsigned int sequence_counter_;
  modbus_t* mb_;
  uint16_t tab_reg_[32];

  double soc_sf_;
  double vol_sf_;
  double bcurrent_sf_;
};
}  // namespace nuv_bms_driver
