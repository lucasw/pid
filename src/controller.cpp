/***************************************************************************/ /**
 * \file controller.cpp
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

// Subscribe to a topic about the state of a dynamic system and calculate
// feedback to
// stabilize it.

#include <pid/controller.h>

using namespace pid_ns;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");

  PidObject my_pid;

  return 0;
}

PidObject::PidObject() : error_(3, 0), filtered_error_(3, 0), error_deriv_(3, 0), filtered_error_deriv_(3, 0)
{
  ros::NodeHandle node;
  ros::NodeHandle node_priv("~");

  while (ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("controller spinning, waiting for time to become non-zero");
    sleep(1);
  }

  // Get params if specified in launch file or as params on command-line, set
  // defaults
  node_priv.param<double>("Kp", Kp_, 1.0);
  node_priv.param<double>("Ki", Ki_, 0.0);
  node_priv.param<double>("Kd", Kd_, 0.0);
  node_priv.param<double>("upper_limit", upper_limit_, 1000.0);
  node_priv.param<double>("lower_limit", lower_limit_, -1000.0);
  node_priv.param<double>("windup_limit", windup_limit_, 1000.0);
  node_priv.param<double>("cutoff_frequency", cutoff_frequency_, -1.0);
  node_priv.param<std::string>("topic_from_controller", topic_from_controller_, "control_effort");
  node_priv.param<std::string>("topic_from_plant", topic_from_plant_, "state");
  node_priv.param<std::string>("setpoint_topic", setpoint_topic_, "setpoint");
  node_priv.param<std::string>("pid_enable_topic", pid_enable_topic_, "pid_enable");
  node_priv.param<double>("max_loop_frequency", max_loop_frequency_, 1.0);
  node_priv.param<double>("min_loop_frequency", min_loop_frequency_, 1000.0);

  // Two parameters to allow for error calculation with discontinous value
  node_priv.param<bool>("angle_error", angle_error_, false);
  node_priv.param<double>("angle_wrap", angle_wrap_, 2.0 * 3.14159);

  // Update params if specified as command-line options, & print settings
  printParameters();
  if (not validateParameters())
    std::cout << "Error: invalid parameter\n";

  // instantiate publishers & subscribers
  control_effort_pub_ = node.advertise<std_msgs::Float64>(topic_from_controller_, 1);

  ros::Subscriber plant_sub_ = node.subscribe(topic_from_plant_, 1, &PidObject::plantStateCallback, this);
  ros::Subscriber setpoint_sub_ = node.subscribe(setpoint_topic_, 1, &PidObject::setpointCallback, this);
  ros::Subscriber pid_enabled_sub_ = node.subscribe(pid_enable_topic_, 1, &PidObject::pidEnableCallback, this);

  // dynamic reconfiguration
  dynamic_reconfigure::Server<pid::PidConfig> config_server;
  dynamic_reconfigure::Server<pid::PidConfig>::CallbackType f;
  f = boost::bind(&PidObject::reconfigureCallback, this, _1, _2);
  config_server.setCallback(f);

  // Respond to inputs until shut down
  while (ros::ok())
  {
    doCalcs();
    ros::spinOnce();

    // Add a small sleep to avoid 100% CPU usage
    ros::Duration(0.001).sleep();
  }
};

void PidObject::setpointCallback(const std_msgs::Float64& setpoint_msg)
{
  setpoint_ = setpoint_msg.data;

  new_state_or_setpt_ = true;
}

void PidObject::plantStateCallback(const std_msgs::Float64& state_msg)
{
  plant_state_ = state_msg.data;

  new_state_or_setpt_ = true;
}

void PidObject::pidEnableCallback(const std_msgs::Bool& pid_enable_msg)
{
  pid_enabled_ = pid_enable_msg.data;
}

void PidObject::getParams(double in, double& value, double& scale)
{
  int digits = 0;
  value = in;
  while ((fabs(value) > 1.0 || fabs(value) < 0.1) && (digits < 2 && digits > -1))
  {
    if (fabs(value) > 1.0)
    {
      value /= 10.0;
      digits++;
    }
    else
    {
      value *= 10.0;
      digits--;
    }
  }
  if (value > 1.0)
    value = 1.0;
  if (value < -1.0)
    value = -1.0;

  scale = pow(10.0, digits);
}

bool PidObject::validateParameters()
{
  if (lower_limit_ > upper_limit_)
  {
    ROS_ERROR("The lower saturation limit cannot be greater than the upper "
              "saturation limit.");
    return (false);
  }

  return true;
}

void PidObject::printParameters()
{
  std::cout << std::endl << "PID PARAMETERS" << std::endl << "-----------------------------------------" << std::endl;
  std::cout << "Kp: " << Kp_ << ",  Ki: " << Ki_ << ",  Kd: " << Kd_ << std::endl;
  if (cutoff_frequency_ == -1)  // If the cutoff frequency was not specified by the user
    std::cout << "LPF cutoff frequency: 1/4 of sampling rate" << std::endl;
  else
    std::cout << "LPF cutoff frequency: " << cutoff_frequency_ << std::endl;
  std::cout << "pid node name: " << ros::this_node::getName() << std::endl;
  std::cout << "Name of topic from controller: " << topic_from_controller_ << std::endl;
  std::cout << "Name of topic from the plant: " << topic_from_plant_ << std::endl;
  std::cout << "Name of setpoint topic: " << setpoint_topic_ << std::endl;
  std::cout << "Integral-windup limit: " << windup_limit_ << std::endl;
  std::cout << "Saturation limits: " << upper_limit_ << "/" << lower_limit_ << std::endl;
  std::cout << "-----------------------------------------" << std::endl;

  return;
}

void PidObject::reconfigureCallback(pid::PidConfig& config, uint32_t level)
{
  if (first_reconfig_)
  {
    getParams(Kp_, config.Kp, config.Kp_scale);
    getParams(Ki_, config.Ki, config.Ki_scale);
    getParams(Kd_, config.Kd, config.Kd_scale);
    first_reconfig_ = false;
    return;  // Ignore the first call to reconfigure which happens at startup
  }

  Kp_ = config.Kp * config.Kp_scale;
  Ki_ = config.Ki * config.Ki_scale;
  Kd_ = config.Kd * config.Kd_scale;
  ROS_INFO("Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f", Kp_, Ki_, Kd_);
}

void PidObject::doCalcs()
{
  // Do fresh calcs if knowledge of the system has changed.
  if (new_state_or_setpt_)
  {
    if (!((Kp_ <= 0. && Ki_ <= 0. && Kd_ <= 0.) ||
          (Kp_ >= 0. && Ki_ >= 0. && Kd_ >= 0.)))  // All 3 gains should have the same sign
      ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for "
               "stability.");

    error_.at(2) = error_.at(1);
    error_.at(1) = error_.at(0);
    error_.at(0) = setpoint_ - plant_state_;  // Current error goes to slot 0

    // If the angle_error param is true, then address discontinuity in error
    // calc.
    // For example, this maintains an angular error between -180:180.
    if (angle_error_)
    {
      while (error_.at(0) < -1.0 * angle_wrap_ / 2.0)
        error_.at(0) += angle_wrap_;
      while (error_.at(0) > angle_wrap_ / 2.0)
        error_.at(0) -= angle_wrap_;

      // The proportional error will flip sign, but the integral error
      // won't and the derivative error will be poorly defined. So,
      // reset them.
      error_.at(2) = 0.;
      error_.at(1) = 0.;
      error_integral_ = 0.;
    }

    // calculate delta_t
    if (!prev_time_.isZero())  // Not first time through the program
    {
      delta_t_ = ros::Time::now() - prev_time_;
      prev_time_ = ros::Time::now();
      if (0 == delta_t_.toSec())
      {
        ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu "
                  "at time: %f",
                  ros::Time::now().toSec());
        return;
      }
    }
    else
    {
      ROS_INFO("prev_time is 0, doing nothing");
      prev_time_ = ros::Time::now();
      return;
    }

    // integrate the error
    error_integral_ += error_.at(0) * delta_t_.toSec();

    // Apply windup limit to limit the size of the integral term
    if (error_integral_ > fabsf(windup_limit_))
      error_integral_ = fabsf(windup_limit_);

    if (error_integral_ < -fabsf(windup_limit_))
      error_integral_ = -fabsf(windup_limit_);

    // My filter reference was Julius O. Smith III, Intro. to Digital Filters
    // With Audio Applications.
    if (cutoff_frequency_ != -1)
    {
      // Check if tan(_) is really small, could cause c = NaN
      tan_filt_ = tan((cutoff_frequency_ * 6.2832) * delta_t_.toSec() / 2);

      // Avoid tan(0) ==> NaN
      if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
        tan_filt_ = -0.01;
      if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
        tan_filt_ = 0.01;

      c_ = 1 / tan_filt_;
    }

    filtered_error_.at(2) = filtered_error_.at(1);
    filtered_error_.at(1) = filtered_error_.at(0);
    filtered_error_.at(0) = (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_.at(2) + 2 * error_.at(1) + error_.at(0) -
                                                                (c_ * c_ - 1.414 * c_ + 1) * filtered_error_.at(2) -
                                                                (-2 * c_ * c_ + 2) * filtered_error_.at(1));

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_.at(2) = error_deriv_.at(1);
    error_deriv_.at(1) = error_deriv_.at(0);
    error_deriv_.at(0) = (error_.at(0) - error_.at(1)) / delta_t_.toSec();

    filtered_error_deriv_.at(2) = filtered_error_deriv_.at(1);
    filtered_error_deriv_.at(1) = filtered_error_deriv_.at(0);

    filtered_error_deriv_.at(0) =
        (1 / (1 + c_ * c_ + 1.414 * c_)) *
        (error_deriv_.at(2) + 2 * error_deriv_.at(1) + error_deriv_.at(0) -
         (c_ * c_ - 1.414 * c_ + 1) * filtered_error_deriv_.at(2) - (-2 * c_ * c_ + 2) * filtered_error_deriv_.at(1));

    // calculate the control effort
    proportional_ = Kp_ * filtered_error_.at(0);
    integral_ = Ki_ * error_integral_;
    derivative_ = Kd_ * filtered_error_deriv_.at(0);
    control_effort_ = proportional_ + integral_ + derivative_;

    // Apply saturation limits
    if (control_effort_ > upper_limit_)
      control_effort_ = upper_limit_;
    else if (control_effort_ < lower_limit_)
      control_effort_ = lower_limit_;

    // Publish the stabilizing control effort if the controller is enabled
    if (pid_enabled_)
    {
      control_msg_.data = control_effort_;
      control_effort_pub_.publish(control_msg_);
    }
    else
      error_integral_ = 0.0;
  }

  new_state_or_setpt_ = false;
}