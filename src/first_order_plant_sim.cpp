// This file simulates a simple dynamic system, publishes its state,
// and subscribes to a 'control_effort' topic. The control effort is used
// to stabilize the plant.

#include "pid/first_order_plant_header.h"

double plant_setpoint = -1.0;

// Callback when something is published on 'control_effort'
void plantControlCallback(const pid::controller_msg& u_msg)
{
  //ROS_INFO("I heard: [%f]", u_msg.u);

  // Define the stabilizing control effort
  u = u_msg.u;
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting simulation of a first-order, single-input plant.");
  ros::init(argc, argv, "plant");
  
  ros::NodeHandle plant_node;

  // Declare a new message variable
  pid::plant_msg  msg;

  // Initial conditions -- these were defined in the header file
  msg.x = x_IC;
  msg.t = t_IC;
  msg.setpoint = plant_setpoint;

  // Publish a plant.msg
  ros::Publisher plant_state_pub = plant_node.advertise<pid::plant_msg>("state", 1);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub = plant_node.subscribe("control_effort", 1, plantControlCallback );
  
  double x_dot= 0;

  int loop_counter = 0;
  int setpoint_change_rate = 500; // Number of loops between negating setpoint
  ros::Rate loop_rate(1/delta_t); // Control rate in Hz


  while (ros::ok())
  {
    ROS_INFO("x1: %f   setpt: %f", msg.x, msg.setpoint);

    plant_state_pub.publish(msg);


    // Update the plant.
    x_dot = 0.1*msg.x+u;
    msg.x = msg.x+x_dot*delta_t;
    msg.t = msg.t+delta_t;
    msg.setpoint = plant_setpoint;

    // periodically negate the setpoint
    if (++loop_counter > setpoint_change_rate)
    {
      ROS_INFO("Negating setpoint");
      plant_setpoint = 0 - plant_setpoint;
      loop_counter = 0;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

