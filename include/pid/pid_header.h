#pragma once

#include "ros/ros.h"
#include "math.h"
#include <stdio.h>
#include <string>
#include <iostream>
using namespace std;

#include "pid/plant_msg.h"
#include "pid/controller_msg.h"

float prev_time = 0.0;
float delta_t = 0.0;
float error_integral = 0.0;

float Kp, Ki, Kd, rate; //Rate in Hz.
float cutoff_frequency= -1.0; // Cutoff frequency for the derivative calculation in Hz. Negative-> Has not been set by the user yet, so use a default.

float ul=1000., ll=-1000.; // Upper and lower saturation limits
float anti_w = 1000.0; // Anti-windup term. Limits the absolute value of the integral term.

vector<float> error(3);
vector<float> filtered_error(3);
vector<float> error_deriv(3);
vector<float> filtered_error_deriv(3);
int loop_counter = 0; // Counts # of times through the control loop. Used to start taking a derivative after 2 rounds

// Get input from the command line
string topic_from_controller = "control_effort";
string topic_from_plant = "state";
string node_name = "pid_node";

pid::controller_msg  u_msg;

////////////////
// Functions
////////////////
void check_user_input(int& argc, char** argv);
void chatterCallback(const pid::plant_msg& msg);
