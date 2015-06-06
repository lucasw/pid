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

vector<float> error(3);
vector<float> filtered_error(3);
vector<float> error_deriv(3);
vector<float> filtered_error_deriv(3);
int loop_counter = 0; // Counts # of times through the control loop. Used to start taking a derivative after 2 rounds

pid::controller_msg  u_msg;

void check_user_input(int& argc, char** argv, float& Kp, float& Ki, float& Kd, float& rate, string& topic_from_controller, string& topic_from_plant, string& node_name, float& ul, float& ll);
void chatterCallback(const pid::plant_msg& msg);
