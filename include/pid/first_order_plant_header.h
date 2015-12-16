#include "ros/ros.h"

// Header for 'plant_msg.msg'
#include "pid/plant_msg.h"

// Header for controller_msg.msg
#include "pid/controller_msg.h"

/////////////////////////////////////////////
// Variables -- Make changes here.
/////////////////////////////////////////////

// Initial conditions
static const double x_IC  = 4.7;
static const double t_IC = 0.0;

double delta_t = 0.01; // control period in seconds

// Global so it can be passed from the callback fxn to main
static double u = 0.0;


/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

