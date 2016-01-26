/***************************************************************************//**
* \file controller.cpp
*
* \brief Simple PID controller with dynamic reconfigure and diagnostics
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

// Subscribe to a topic about the state of a dynamic system and calculate feedback to
// stabilize it.

#include <pid/controller.h>
#include <pid/PidConfig.h>

#include <dynamic_reconfigure/server.h>
#include <ros/time.h>

void setpoint_callback(const std_msgs::Float64& setpoint_msg)
{
  setpoint = setpoint_msg.data;
  ROS_INFO("Received new setpoint: %f", setpoint);
}

void plant_state_callback(const std_msgs::Float64& state_msg)
{
  plant_state = state_msg.data;

  error.at(2) = error.at(1);
  error.at(1) = error.at(0);
  error.at(0) = setpoint - plant_state; // Current error goes to slot 0

  // calculate delta_t
  if (!prev_time.isZero()) // Not first time through the program  
  {
    delta_t = ros::Time::now() - prev_time;
  }
  
  prev_time = ros::Time::now();

  // integrate the error
  error_integral += error.at(0) * delta_t.toSec();

  // Apply windup limit to limit the size of the integral term
  if ( error_integral > fabsf(windup_limit))
    error_integral = fabsf(windup_limit);

  if ( error_integral < -fabsf(windup_limit))
    error_integral = -fabsf(windup_limit);

  // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
  double c;
  if (cutoff_frequency == -1)
    c = 1.0; // Default to a cut-off frequency at one-fourth of the sampling rate
  else
    c = 1/tan( (cutoff_frequency*6.2832)*delta_t.toSec()/2 );
 
  filtered_error.at(2) = filtered_error.at(1);
  filtered_error.at(1) = filtered_error.at(0); 
  filtered_error.at(0) = (1/(1+c*c+1.414*c))*(error.at(2)+2*error.at(1)+error.at(0)-(2-1.414)*filtered_error.at(2));
  //std::cout<<"Error: "<< error.at(0) << "   Filtered_error: "<< filtered_error.at(0)<<std::endl;

  // Take derivative of error
  // First the raw, unfiltered data:
  error_deriv.at(2) = error_deriv.at(1);
  error_deriv.at(1) = error_deriv.at(0);
  error_deriv.at(0) = (error.at(0)-error.at(1))/delta_t.toSec();

  filtered_error_deriv.at(2) = filtered_error_deriv.at(1);
  filtered_error_deriv.at(1) = filtered_error_deriv.at(0);

  if ( loop_counter>2 ) // Let some data accumulate
    filtered_error_deriv.at(0) = (1/(1+c*c+1.414*c))*(error_deriv.at(2)+2*error_deriv.at(1)+error_deriv.at(0)-(2-1.414)*filtered_error_deriv.at(2));
  else
    loop_counter++;

  //std::cout<<"Filtered error derivative: "<< filtered_error_deriv.at(0)<<std::endl;

  // calculate the control effort
  proportional = Kp * filtered_error.at(0);
  integral = Ki * error_integral;
  derivative = Kd * filtered_error_deriv.at(0);
  control_effort = proportional + integral + derivative;

  // Apply saturation limits
  if (control_effort > upper_limit)
    control_effort = upper_limit;
  if (control_effort < lower_limit)
    control_effort = lower_limit;
  //std::cout << "control_effort: " << control_effort << std::endl;

  ++measurements_received;
  diags->freq_status.tick();
  diags->diag_updater.update();

  // Publish the stabilizing control effort
  control_msg.data = control_effort;
  control_effort_pub.publish(control_msg);

  return;
}

bool first_reconfig = true;

void reconfigure_callback(pid::PidConfig &config, uint32_t level)
{
  if (first_reconfig)
  {
    first_reconfig = false;
    return;     // Ignore the first call to reconfigure which happens at startup
  }

  Kp = config.Kp * config.Kp_scale;
  Ki = config.Ki * config.Ki_scale;
  Kd = config.Kd * config.Kd_scale;
  ROS_INFO("Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f", Kp, Ki, Kd);
}

void get_pid_diag_status(diagnostic_updater::DiagnosticStatusWrapper& pid_diag_status)
{
  pid_diag_status.summary(diagnostic_msgs::DiagnosticStatus::OK, "PID controller nominal");
  pid_diag_status.add("Setpoint", setpoint);
  pid_diag_status.add("Pid Controller input", plant_state);
  pid_diag_status.add("Error", error.at(0));
  pid_diag_status.add("Control output effort", control_effort);
  pid_diag_status.add("Proportional effort", proportional);
  pid_diag_status.add("Integral effort", integral);
  pid_diag_status.add("Derivative effort", derivative);
  pid_diag_status.add("Measurements received", measurements_received);
}

void usage()
{
  std::cout << std::endl;
  std::cout << "Usage: controller [ Kp Ki Kd rate [<command-line options>]" << std::endl;
  std::cout << "Example: rosrun pid controller 1.1 2.2 3.3 100 -fc 100 -nn pid_node_name" << std::endl;
  std::cout << std::endl;
  std::cout << "All arguments are optional and override parameters. Defaults are provided." << std::endl;
  std::cout << "Optional arguments:" << std::endl << std::endl;
  std::cout << "Kp (default: 1), Ki (default: 0), Kd (default: 0), rate (default: 50)\n";
  std::cout << "-fc Filter Cutoff frequency [Hz]" << std::endl;
  std::cout << "-tfc name of Topic From Controller" << std::endl;
  std::cout << "-ttc name of Topic From Plant" << std::endl;
  std::cout << "-nn Name of pid Node" << std::endl;
  std::cout << "-ul Upper Limit of control effort, e.g. maximum motor torque" << std::endl;
  std::cout << "-ll Lower Limit of control effort, e.g. minimum motor torque" << std::endl;
  std::cout << "-aw Anti-Windup, i.e. the largest value the integral term can have." << std::endl
            << std::endl;
  std::cout << "Alternatively, provide parameters ~Kp, ~Ki, ~Kd, ~cutoff_frequency," << std::endl 
       << "~upper_limit, ~lower_limit, ~windup_limit," << std::endl
       << "~topic_from_controller, ~topic_from_plant, ~node_name"
       << std::endl;
}

void check_user_input(int& argc, char** argv)
{
  // Remove any arguments that are added by roslaunch
  ros::V_string args_out; //Vector of strings
  ros::removeROSArgs(argc, argv, args_out);

  // If no arguments provided, print usage
  if (1 == args_out.size())
    usage();

  // First 4 arguments (Kp, Ki, Kd, rate) are positional and if provided, must be provided
  // before any others are provided
  std::stringstream ss;
  if (1 < args_out.size())    // if have at least one command-line arg
  {
    ss.str(std::string()); // Clear the variable
    ss.clear();
    ss.str( args_out.at(1) ); // Read Kp
    ss >> Kp;
  }

  if (2 < args_out.size())    // if have at least 2 command-line args
  {
    ss.str(std::string()); // Clear the variable
    ss.clear();
    ss.str(args_out.at(2)); // Read Ki
    ss >> Ki;
  }

  if (3 < args_out.size())    // if have at least 3 command-line args
  {
    ss.str(std::string()); // Clear the variable
    ss.clear();
    ss << args_out.at(3); // Read Kd
    ss >> Kd;
  }

  if (4 < args_out.size())    // if have at least 4 command-line args
  {
    ss.str(std::string()); // Clear the variable
    ss.clear();
    ss << args_out.at(4); // Read rate
    ss >> rate;
  }


  // Scan for any optional arguments
  // Every other argument is a tag

  char tag [] = {'x','x','x','x'};
  
  if (args_out.size()>5) // If there were optional arguments
  {
    for ( int i=5; i< args_out.size()-1; i=i+2)
    {
      // Read the tag
      ss.str(std::string()); // Clear the variable
      ss.clear();
      ss << args_out.at(i);
      ss >> tag;

      //sscanf(args_out.at(i),"%s", tag);

      // Cutoff frequency
      if ( !strncmp(tag,"-fc",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%lf",&cutoff_frequency);

      // Name of topic from controller
      if ( !strncmp(tag,"-tfc",4) ) // Compare first 4 chars
        topic_from_controller = std::string(argv[i+1]);

      // Name of topic to controller
      if ( !strncmp(tag,"-tfp",4) ) // Compare first 4 chars
        topic_from_plant = std::string(argv[i+1]);

      // Name of pid node
      if ( !strncmp(tag,"-nn",3) ) // Compare first 3 chars
        node_name = std::string(argv[i+1]);

      // Upper saturation limit
      if ( !strncmp(tag,"-ul",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%lf",&upper_limit);

      // Lower saturation limit
      if ( !strncmp(tag,"-ll",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%lf",&lower_limit);

      // Anti-windup
      // Limit the maximum size that the integral term can have
      if ( !strncmp(tag,"-aw",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%lf",&windup_limit);
    }
  }

  ////////////////////////////////////
  // Error checking
  ////////////////////////////////////

  if ( rate <= 0 )
  {
    ROS_ERROR("Enter a positive value for the loop rate.");
    exit(1);
  }

  if ( lower_limit > upper_limit )
  {
    ROS_ERROR("The lower saturation limit cannot be greater than the upper saturation limit.");
    exit(1);
  }
  return;
}

  ////////////////////////////////////
  // Display parameters
  ////////////////////////////////////
void print_parameters()
{
  std::cout<< std::endl<<"PID PARAMETERS"<<std::endl<<"-----------------------------------------"<<std::endl;
  std::cout << "Kp: " << Kp << ",  Ki: " << Ki << ",  Kd: " << Kd << ",  Loop rate [Hz]: "
            << rate << std::endl;
  if ( cutoff_frequency== -1) // If the cutoff frequency was not specified by the user
    std::cout<<"LPF cutoff frequency: 1/4 of sampling rate"<<std::endl;
  else
    std::cout<<"LPF cutoff frequency: "<< cutoff_frequency << std::endl;
  std::cout << "pid node name: " << ros::this_node::getName() << std::endl;
  std::cout << "Name of topic from controller: " << topic_from_controller << std::endl;
  std::cout << "Name of topic from the plant: " << topic_from_plant << std::endl;
  std::cout << "Name of setpoint topic: " << setpoint_topic << std::endl;
  std::cout << "Integral-windup limit: " << windup_limit << std::endl;
  std::cout << "Saturation limits: " << upper_limit << "/" << lower_limit << std::endl
            << "-----------------------------------------" << std::endl;

  return;
}

////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ROS_INFO("Starting pid with node name %s", node_name.c_str());

  // Initialize ROS stuff
  ros::init(argc, argv, node_name);     // Note node_name can be overidden by launch file
  ros::NodeHandle node;
  ros::NodeHandle node_priv("~");

  // Get params if specified in launch file or as params on command-line, set defaults
  node_priv.param<double>("Kp", Kp, 1.0);
  node_priv.param<double>("Ki", Ki, 0.0);
  node_priv.param<double>("Kd", Kd, 0.0);
  node_priv.param<double>("rate", rate, 50.0);
  node_priv.param<double>("upper_limit", upper_limit, 1000.0);
  node_priv.param<double>("lower_limit", lower_limit, -1000.0);
  node_priv.param<double>("windup_limit", windup_limit, 1000.0);
  node_priv.param<double>("cutoff_frequency", cutoff_frequency, -1.0);
  node_priv.param<std::string>("topic_from_controller", topic_from_controller, "control_effort");
  node_priv.param<std::string>("topic_from_plant", topic_from_plant, "state");
  node_priv.param<std::string>("setpoint_topic", setpoint_topic, "setpoint");

  // Update params if specified as command-line options, & print settings
  check_user_input(argc, argv);
  print_parameters();

  // instantiate publishers & subscribers
  control_effort_pub = node.advertise<std_msgs::Float64>(topic_from_controller, 1);

  ros::Subscriber sub = node.subscribe(topic_from_plant, 1, plant_state_callback );
  ros::Subscriber setpoint_sub = node.subscribe(setpoint_topic, 1, setpoint_callback );

  // configure dynamic reconfiguration
  dynamic_reconfigure::Server<pid::PidConfig> config_server;
  dynamic_reconfigure::Server<pid::PidConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1, _2);
  config_server.setCallback(f);

  // initialize diagnostics updaters
  diags = new PidControllerDiags;

  diags->diag_updater.setHardwareID(node_name);
  diags->diag_updater.add(diags->freq_status);
  diags->diag_updater.add("PID status", get_pid_diag_status);

  // Respond to inputs until shut down
  ros::spin();

  return 0;
}

