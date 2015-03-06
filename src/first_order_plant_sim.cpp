// This file simulates a simple dynamic system, publishes its state,
// and subscribes to a 'control_effort' topic. The control effort is used
// to stabilize the plant.

#include "pid/first_order_plant_header.h"


int main(int argc, char **argv)
{
  ROS_INFO("Starting simulation of a first-order, single-input plant.");
  ros::init(argc, argv, "plant");
  
  ros::NodeHandle plant_node;

  // Declare a new message variable
  lyap_control::plant_msg  msg;

  // Initial conditions -- these were defined in the header file
  msg.x[0] = x_IC[0];
  msg.t = t_IC;
  msg.setpoint[0] = setpoint[0];

  // Publish a plant.msg
  ros::Publisher chatter_pub = plant_node.advertise<lyap_control::plant_msg>("state", 1);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub = plant_node.subscribe("control_effort", 1, chatterCallback );
  
  double x_dot [num_states] = {0};

  ros::Rate loop_rate(1/delta_t); // Control rate in Hz


  while (ros::ok())
  {
    ROS_INFO("x1: %f   setpt: %f", msg.x[0], msg.setpoint[0]);

    chatter_pub.publish(msg);


    // Update the plant.
    x_dot[0] = 0.1*msg.x[0]+u[0];
    msg.x[0] = msg.x[0]+x_dot[0]*delta_t;
    msg.t = msg.t+delta_t;

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

// Callback when something is published on 'control_effort'
void chatterCallback(const lyap_control::controller_msg& u_msg)
{
  //ROS_INFO("I heard: [%f]", u_msg.u[0]);

  // Define the stabilizing control effort
  u[0] = u_msg.u[0];
}
