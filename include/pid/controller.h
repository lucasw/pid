/***************************************************************************//**
* \file controller.h
*
* \brief Simple PID controller with dynamic reconfigure
* \author Andy Zelenak
* \date March 8, 2015
*
* \section license License (BSD-3)
* Copyright (c) 2015, Andy Zelenak\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <dynamic_reconfigure/server.h>
#include <iostream>
#include "math.h"
#include <pid/PidConfig.h>
#include "ros/ros.h"
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <string>

namespace pid
{
  // Primary PID controller input & output variables
  double plant_state;                 // current output of plant
  double control_effort;              // output of pid controller
  double setpoint = 0;                // desired output of plant
  bool pid_enabled = true;            // PID is enabled to run

  ros::Time prev_time;
  ros::Duration delta_t;
  bool first_reconfig = true;

  double error_integral = 0;
  double proportional = 0;         // proportional term of output
  double integral = 0;             // integral term of output
  double derivative = 0;           // derivative term of output

  // PID gains
  double Kp = 0, Ki = 0, Kd = 0;

  // Parameters for error calc. with disconinuous input
  bool angle_error = false;
  double angle_wrap = 2.0*3.14159;

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency = -1; 

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
  // 1/4 of the sample rate.
  double c=1.;

  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt = 1.;

  // Upper and lower saturation limits
  double upper_limit =  1000, lower_limit = -1000;

  // Anti-windup term. Limits the absolute value of the integral term.
  double windup_limit = 1000;

  // Initialize filter data with zeros
  std::vector<double> error(3, 0), filtered_error(3, 0), error_deriv(3, 0), filtered_error_deriv(3, 0);

  // Topic and node names and message objects
  ros::Publisher control_effort_pub;

  std::string topic_from_controller, topic_from_plant, setpoint_topic, pid_enable_topic, node_name = "pid_node";

  std_msgs::Float64 control_msg, state_msg;

  // Diagnostic objects
  double min_loop_frequency = 1, max_loop_frequency = 1000;
  int measurements_received = 0;

} // end pid namespace

#endif
