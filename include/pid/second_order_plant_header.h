#include "ros/ros.h"

// Header for 'plant_msg.msg'
#include "pid/plant_msg.h"

// Header for controller_msg.msg
#include "pid/controller_msg.h"

/////////////////////////////////////////////
// User-defined variables -- Make changes here.
/////////////////////////////////////////////

static const int num_states=2;
static const int num_inputs=2;

// Initial conditions
static const double x_IC [num_states] = {0.1, -0.2};
static const double t_IC = 0.0;
static const double setpoint [num_states] = {1.0, -2.0};

double delta_t = 0.01; // control period in seconds

// Global so it can be passed from the callback fxn to main
static double u[num_inputs] = {0.0, 0.0};


/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

void chatterCallback(const pid::controller_msg& u_msg);
