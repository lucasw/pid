
// Subscribe to a topic about the state of a dynamic system and calculate feedback to
// stabilize it.
// Should run at a faster loop rate than the plant.

#include "pid/pid_header.h"

////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  // Get input from the command line
  float Kp, Ki, Kd, rate; //Rate in Hz.
  string topic_from_controller, topic_from_plant, node_name;

  check_user_input(argc,argv, Kp,Ki,Kd, rate, topic_from_controller, topic_from_plant, node_name);

  // Initialize ROS stuff
  ros::init(argc, argv, node_name);
  ros::NodeHandle node;
  pid::controller_msg  u_msg;
  cout<<rate<<endl;
  ros::Rate loop_rate(rate); // Control frequency in Hz

  // Publish on "control_effort" topic
  ros::Publisher chatter_pub = node.advertise<pid::controller_msg>(topic_from_controller, 1);

  // Subscribe to "state" topic
  ros::Subscriber sub = node.subscribe(topic_from_plant, 1, chatterCallback );

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
  ROS_INFO("I heard: %f", msg.x[0]);

  // calculate error

  // calculate delta_t

  // integrate error

  // take derivative of error

  // calculate the control effort

  // publish it
}

void check_user_input(int& argc, char** argv, float& Kp, float& Ki, float& Kd, float& rate, string& topic_from_controller, string& topic_from_plant, string& node_name)
{
  if ( argc<5 || argc>8 )
  {
    ROS_ERROR("Incorrect input arguments. Please follow the rosrun command with Kp, Ki, Kd, loop_rate. Custom topic names and a node name are optional.");
    ROS_ERROR("Example: rosrun pid controller 1.1 2.2 3.3 100 topic_from_controller topic_from_plant pid_node");
    exit(1);
  }

  sscanf(argv[1],"%f",&Kp); // Read Kp
  sscanf(argv[2],"%f",&Ki);
  sscanf(argv[3],"%f",&Kd);
  sscanf(argv[4],"%f",&rate);

  if (argc==8)
  {
    topic_from_controller = string(argv[5]);
    topic_from_plant = string(argv[6]);
    node_name = string(argv[7]);
  }
  else
  {
    topic_from_controller = "control_effort";
    topic_from_plant = "state";
    node_name = "pid_node";
  }

  if ( rate <= 0 )
  {
    ROS_ERROR("Enter a positive value for the loop rate.");
    exit(1);
  }

  cout<<"Kp: "<<Kp<<" Ki: "<<Ki<<" Kd: "<<Kd<<" Loop rate: "<<rate<<endl;
}
