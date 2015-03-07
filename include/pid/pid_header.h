#pragma once

#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
using namespace std;


#include "pid/plant_msg.h"
#include "pid/controller_msg.h"

void check_user_input(int& argc, char** argv, float& Kp, float& Ki, float& Kd, float& rate);
void chatterCallback(const pid::plant_msg& msg);
