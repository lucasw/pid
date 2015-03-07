#pragma once

#include "ros/ros.h"
#include <iostream>
using namespace std;


#include "pid/plant_msg.h"
#include "pid/controller_msg.h"

void get_user_input(double& Kp, double& Ki, double& Kd, double& rate);
void chatterCallback(const pid::plant_msg& msg);
