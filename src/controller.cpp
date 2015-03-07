
// Subscribe to a topic about the state of a dynamic system and calculate feedback to
// stabilize it.
// Should run at a faster loop rate than the plant.

#include "pid/pid_header.h"

////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  double Kp=0.0;
  double Ki=0.0;
  double Kd=0.0;
  double rate= 1.0; //in Hz.

  get_user_input(Kp,Ki,Kd,rate);

  // Initialize ROS stuff
  ros::init(argc, argv, "controller");
  ros::NodeHandle controller_node;
  pid::controller_msg  u_msg;
  ros::Rate loop_rate(rate); // Control frequency in Hz

  // Publish on "control_effort" topic
  ros::Publisher chatter_pub = controller_node.advertise<pid::controller_msg>("control_effort", 1);

  // Subscribe to "state" topic
  ros::Subscriber sub = controller_node.subscribe("state", 1, chatterCallback );

  // MAIN LOOP
  while( ros::ok() )
  {
    ros::spinOnce();

    // Publish the stabilizing control effort
    chatter_pub.publish(u_msg);

    loop_rate.sleep();
  }

  return 0;
}

void chatterCallback(const pid::plant_msg& msg)
{
  // Translate a message from the plant    

  // calculate error

  // calculate delta_t

  // integrate error

  // take derivative of error

  // calculate the control effort

  // publish it
}

void get_user_input(double& Kp, double& Ki, double& Kd, double& rate)
{
  cout<<"Enter the proportional gain, Kp: "<<endl;
  if ( !(cin >> Kp) )
  {
    ROS_ERROR("Invalid input.");
    exit(1);
  }

  cout<<"Enter the integral gain, Ki: "<<endl;
  if ( !(cin >> Ki) )
  {
    ROS_ERROR("Invalid input.");
    exit(1);
  }

  cout<<"Enter the derivative gain, Kd: "<<endl;
  if ( !(cin >> Kd) )
  {
    ROS_ERROR("Invalid input.");
    exit(1);
  }

  cout<<"Enter the loop rate: "<<endl;
  if ( !(cin >> rate) )
  {
    ROS_ERROR("Invalid input.");
    exit(1);
  }

  cout<<"Kp: "<<Kp<<" Ki: "<<Ki<<" Kd: "<<Kd<<" Loop rate: "<<rate<<endl;
}
