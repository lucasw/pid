/***************************************************************************//**
* \file autotune.cpp
*
* \brief Autotune a PID controller with the Ziegler Nichols method
* \author Andy Zelenak
* \date October 25, 2016
*
* \section license License (BSD-3)
* Copyright (c) 2016, Andy Zelenak\n
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

// Use the Ziegler Nichols Method to calculate reasonable PID gains.
// The ZN Method is based on setting Ki & Kd to zero, then cranking up Kp until
// oscillations are observed.
// This node varies Kp through the range of -10 to 10 until oscillations are just barely observed,
// then calculates the other parameters automatically.
// See https://en.wikipedia.org/wiki/PID_controller

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <math.h>

// Use dynamic_reconfigure to adjust Kp, Ki, and Kd
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

void setKiKdToZero();
void setKp(double Kp);
void setpoint_callback(const std_msgs::Float64& setpoint_msg);
void state_callback(const std_msgs::Float64& state_msg);

namespace autotune
{
 double Ku = 0.;
 double Tu = 0.;
 double setpoint = 0.;
 double state = 0.;
 std::string nameSpc = "/left_wheel_pid/";
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "autotune_node");
  ros::NodeHandle autotuneNode;
  ros::start();
  ros::Subscriber setpoint_sub = autotuneNode.subscribe("/setpoint", 1, setpoint_callback );
  ros::Subscriber state_sub = autotuneNode.subscribe("/state", 1, state_callback );
  ros::Rate loopRate(1000); // Collect data on the performance of each parameter set for X ms
  
  // Set Ki and Kd to zero for the ZN method with dynamic_reconfigure
  setKiKdToZero();

  // Define how rapidly the value of Kp is varied, and the max/min values to try
  double Kp_max = 10.;
  double Kp_min = 0.;
  double Kp_step = .5;
    
  for (double Kp = Kp_min; Kp <= Kp_max; Kp += Kp_step)
  {
    // Get the error sign.
    // Need to wait for new setpoint/state msgs
    ros::topic::waitForMessage<std_msgs::Float64>("setpoint");
    ros::topic::waitForMessage<std_msgs::Float64>("state");
    double errorSign = (autotune::setpoint - autotune::state) / fabs(autotune::setpoint - autotune::state); //Sign of the error
    
    // Try a new Kp.
    setKp(Kp);
    
    ros::spinOnce();
    loopRate.sleep();
     
    // Did the system oscillate about the setpoint? If so, Kp~Ku.
    // It oscillates if the sign of the error changes
    // Get a fresh state message (assume the setpoint remains the same... nearly always true)
    ros::topic::waitForMessage<std_msgs::Float64>("state");
    double newErrorSign = (autotune::setpoint - autotune::state) / fabs(autotune::setpoint - autotune::state); //Sign of the error
    if ( errorSign != newErrorSign )
    {
      ROS_INFO_STREAM("Oscillation occurred");
      //autotune::Ku = Kp;
	    
      // Now calculate the period of oscillation (Tu)
      //autotune::Tu = 1.0;
      
      //goto FOUND_KU;
    }
  }
  
FOUND_KU:
  // Now that Ku is known, the other parameters can be calculated
  double Kp_ZN = 0.6*autotune::Ku;
  double Ki_ZN = 1.2*autotune::Ku/autotune::Tu;
  double Kd_ZN = 3.*autotune::Ku*autotune::Tu/40.;
  
  ROS_INFO_STREAM("The suggested parameters are: ");
  ROS_INFO_STREAM("Kp  "<< Kp_ZN);
  ROS_INFO_STREAM("Ki  "<< Ki_ZN);
  ROS_INFO_STREAM("Kd  "<< Kd_ZN);

  ros::shutdown();
  return 0;
}

// Set Ki and Kd to zero with dynamic_reconfigure
void setKiKdToZero()
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config config;
  double_param.name = "Ki";
  double_param.value = 0.0;
  config.doubles.push_back(double_param);
  double_param.name = "Kd";
  double_param.value = 0.0;
  config.doubles.push_back(double_param);
  srv_req.config = config;
  ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);
}

// Set Kp with dynamic_reconfigure
void setKp(double Kp)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config config;
  
  // A blank service call to get the current parameters into srv_resp
  ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);
  
  double_param.name = "Kp";
  double_param.value = Kp / srv_resp.config.doubles.at(0).value; // Adjust for the scale slider on the GUI
  config.doubles.push_back(double_param);
  srv_req.config = config;
  ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);
}

void setpoint_callback(const std_msgs::Float64& setpoint_msg)
{
  autotune::setpoint = setpoint_msg.data;
}

void state_callback(const std_msgs::Float64& state_msg)
{
  autotune::state = state_msg.data;
}