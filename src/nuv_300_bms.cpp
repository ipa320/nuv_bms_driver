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

#include <nuv_bms_driver/nuv_300_bms.h>
#include <ros/ros.h>

namespace nuv_bms_driver
{

  Nuv300BMS::Nuv300BMS() : sequence_counter_(0)
  {
    // parameters
    ros::NodeHandle pnh("~");
    ROS_DEBUG_STREAM("========== Nuv300BMS Parameters ==========");
    double loop_rate;
    pnh.param("loop_rate", loop_rate, 10.);
    ROS_DEBUG_STREAM("loop_rate: " << loop_rate);
    pnh.param("bms_ip_address", bms_ip_address_);
    ROS_DEBUG_STREAM("bms_ip_address: " << bms_ip_address_);
    pnh.param("bms_port", bms_port_, 502);
    ROS_DEBUG_STREAM("bms_port: " << bms_port_);

    // setup ros publisher
    power_state_pub_ = pnh.advertise<cob_msgs::PowerState>("power_state", 0);

    // connect to modbus
    mb_ = modbus_new_tcp(bms_ip_address_.c_str(), bms_port_);
    modbus_connect(mb_);

    // receive the scale factors
    if (modbus_read_registers(mb_, 40092, 1, tab_reg_) == -1)
     ROS_ERROR_STREAM("Modbus error: " << modbus_strerror(errno));
    soc_sf_ = pow(10, static_cast<double>(static_cast<int16_t>(tab_reg_[0])));
    if (modbus_read_registers(mb_, 40113, 1, tab_reg_) == -1)
     ROS_ERROR_STREAM("Modbus error: " << modbus_strerror(errno));
    vol_sf_ = pow(10, static_cast<double>(static_cast<int16_t>(tab_reg_[0])));
    if (modbus_read_registers(mb_, 40132, 1, tab_reg_) == -1)
     ROS_ERROR_STREAM("Modbus error: " << modbus_strerror(errno));
    bcurrent_sf_ = pow(10, static_cast<double>(static_cast<int16_t>(tab_reg_[0])));

    ROS_DEBUG_STREAM("soc_sf=" << soc_sf_ << "   vol_sf=" << vol_sf_ << "   bcurrent_sf=" << bcurrent_sf_);


    timer_ = pnh.createTimer(ros::Rate(loop_rate), &Nuv300BMS::timer_callback, this);

  }


Nuv300BMS::~Nuv300BMS()
{
    modbus_close(mb_);
    modbus_free(mb_);

}
void Nuv300BMS::timer_callback(const ros::TimerEvent& event)
{
  // From: https://www.nuvationenergy.com/technical-resources
  // Communication Protocol Reference Guide:
  // https://www.nuvationenergy.com/sites/default/files/Nuvation-Energy-Images/Technical-Resources/Nuvation-BMS-Communication-Protocol-Reference-Guide.pdf
  // Further specification:
  // http://mesastandards.org/wp-content/uploads/2015/10/Energy-Storage-Information-Models_D3-2015-10-26-Update.xlsx
  //
  // Model	Block	Point Name		Address		  Type		Unit	Scale Factor	Purpose
  // 801		Fixed	SoC				    40081		    uint16	%		  SoC_SF			  BMS State of Charge
  // 801		Fixed	SoC_SF			  40092		    sunssf			  					    Scale Factor for SoC
  // 802		Fixed	Vol				    40105		    uint16	V		  Vol_SF			  External DC voltage of the battery system
  // 802		Fixed	Vol_SF			  40113		    sunssf			  					    Scale Factor for Vol
  // 803		Fixed	BTotDCCur		  40127		    int16		A		  BCurrent_SF		Total DC current of the battery system
  // 803		Fixed	BCurrent_SF		40132		    sunssf			  					    Scale Factor for BTotDCCur
  //?803		Repeat	StrCur			40137+Index	int16		A		  BCurrent_SF		Current of a stack/string ?
  //
  // The term Index in the Repeating block addresses used in the above table refers to a calculation of
  // Index = Stack Index * Length of Repeating block. By definition, the 803 Repeating block is 16 Modbus registers
  // in length.
  cob_msgs::PowerState msg;

  // read SoC
  modbus_read_registers(mb_, 40081, 1, tab_reg_);
  msg.relative_remaining_capacity =static_cast<double>(tab_reg_[0]) * soc_sf_;

  // read Vol
  modbus_read_registers(mb_, 40105, 1, tab_reg_);
  msg.voltage =static_cast<double>(tab_reg_[0]) * vol_sf_;

  // read BTotDCCur
  modbus_read_registers(mb_, 40127, 1, tab_reg_);
  msg.current =static_cast<double>(static_cast<int16_t>(tab_reg_[0])) * bcurrent_sf_;

  // publish PowerState message
  msg.header.seq = sequence_counter_++;
  msg.header.stamp = ros::Time::now();
  msg.charging = (msg.current < 0. ? true : false);  // todo: check whether this assumption is correct
  msg.remaining_capacity = 0.;
  power_state_pub_.publish(msg);
}

}
