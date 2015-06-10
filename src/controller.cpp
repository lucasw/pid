
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
  string topic_from_controller = "control_effort";
  string topic_from_plant = "state";
  string node_name = "pid_node";

  check_user_input(argc,argv, Kp,Ki,Kd, rate, topic_from_controller, topic_from_plant, node_name, ul, ll, anti_w);

  // Initialize ROS stuff
  ros::init(argc, argv, node_name);
  ros::NodeHandle node;
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
    //cout<< "Published control effort: "<< u_msg.u<<endl;
    chatter_pub.publish(u_msg);

    loop_rate.sleep();
  }

  return 0;
}

void chatterCallback(const pid::plant_msg& msg)
{
  //ROS_INFO("I heard: %f", msg.x);

  error.at(2) = error.at(1);
  error.at(1) = error.at(0);
  error.at(0) = msg.setpoint-msg.x; // Current error goes to slot 0

  // calculate delta_t
  if (prev_time != 0) // Not first time through the program  
  {
    delta_t = msg.t-prev_time;
  }
  
  prev_time = msg.t;

  // integrate the error
  error_integral += error.at(0)*delta_t;

  // Apply anti-windup to limit the size of the integral term
  if ( error_integral > abs(anti_w) )
    error_integral = abs(anti_w);

  if ( error_integral < -abs(anti_w) )
    error_integral = -abs(anti_w);

  // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
  float c;
  if (cutoff_frequency == -1)
    c = 1.0; // Default to a cut-off frequency at one-fourth of the sampling rate
  else
    c = 1/tan( (cutoff_frequency*6.2832)*delta_t/2 );
 
  filtered_error.at(2) = filtered_error.at(1);
  filtered_error.at(1) = filtered_error.at(0); 
  filtered_error.at(0) = (1/(1+c*c+1.414*c))*(error.at(2)+2*error.at(1)+error.at(0)-(2-1.414)*filtered_error.at(2));
  //cout<<"Error: "<< error.at(0) << "   Filtered_error: "<< filtered_error.at(0)<<endl;

  // Take derivative of error
  // First the raw, unfiltered data:
  error_deriv.at(2) = error_deriv.at(1);
  error_deriv.at(1) = error_deriv.at(0);
  error_deriv.at(0) = (error.at(0)-error.at(1))/delta_t;

  filtered_error_deriv.at(2) = filtered_error_deriv.at(1);
  filtered_error_deriv.at(1) = filtered_error_deriv.at(0);

  if ( loop_counter>2 ) // Let some data accumulate
    filtered_error_deriv.at(0) = (1/(1+c*c+1.414*c))*(error_deriv.at(2)+2*error_deriv.at(1)+error_deriv.at(0)-(2-1.414)*filtered_error_deriv.at(2));
  else
    loop_counter++;

  //cout<<"Filtered error derivative: "<< filtered_error_deriv.at(0)<<endl;

  // calculate the control effort
  u_msg.u = Kp*filtered_error.at(0)+Ki*error_integral+Kd*filtered_error_deriv.at(0);

  // Check saturation limits
  if (u_msg.u>ul)
    u_msg.u = ul;
  if (u_msg.u<ll)
    u_msg.u = ll;
}

void check_user_input(int& argc, char** argv, float& Kp, float& Ki, float& Kd, float& rate, string& topic_from_controller, string& topic_from_plant, string& node_name, float& ul, float& ll, float& anti_w)
{
  if ( (argc<5 || argc>19) || (argc%2 != 1) ) // Wrong # or not an even number of arguments
  {
    ROS_ERROR("Incorrect input arguments. Please follow the rosrun command with Kp, Ki, Kd, loop_rate. A custom filter cutoff frequency, saturation limits, node name, and topic names are optional.");
    cout<<endl;
    cout<<"Example: rosrun pid controller 1.1 2.2 3.3 100 -fc 100 -nn pid_node_name"<<endl;
    cout<<endl;
    cout<<"Optional arguments:"<<endl<<endl;
    cout<<"-fc Filter Cutoff frequency [Hz]"<<endl;
    cout<<"-tfc name of Topic From Controller"<<endl;
    cout<<"-ttc name of Topic From Plant"<<endl;
    cout<<"-nn Name of pid Node"<<endl;
    cout<<"-ul Upper Limit of control effort, e.g. maximum motor torque"<<endl;
    cout<<"-ll Lower Limit of control effort, e.g. minimum motor torque"<<endl;
    cout<<"-aw Anti-Windup, i.e. the largest value the integral term can have."<<endl<<endl;
    exit(1);
  }

  // First 4 arguments are mandatory
  sscanf(argv[1],"%f",&Kp); // Read Kp
  sscanf(argv[2],"%f",&Ki);
  sscanf(argv[3],"%f",&Kd);
  sscanf(argv[4],"%f",&rate);

  // Scan for any optional arguments
  // Every other argument is a tag

  char str_4 [] = {'x','x','x','x'};
  
  if (argc>5) // If there were optional arguments
  {
    for ( int i=5; i< argc-1; i=i+2)
    {
      sscanf(argv[i],"%s", str_4);

      // Cutoff frequency
      if ( !strncmp(str_4,"-fc",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%f",&cutoff_frequency);

      // Name of topic from controller
      if ( !strncmp(str_4,"-tfc",4) ) // Compare first 4 chars
        topic_from_controller = string(argv[i+1]);

      // Name of topic to controller
      if ( !strncmp(str_4,"-tfp",4) ) // Compare first 4 chars
        topic_from_plant = string(argv[i+1]);

      // Name of pid node
      if ( !strncmp(str_4,"-nn",3) ) // Compare first 3 chars
        node_name = string(argv[i+1]);

      // Upper saturation limit
      if ( !strncmp(str_4,"-ul",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%f",&ul);

      // Lower saturation limit
      if ( !strncmp(str_4,"-ll",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%f",&ll);

      // Anti-windup
      // Limit the maximum size that the integral term can have
      if ( !strncmp(str_4,"-aw",3) ) // Compare first 3 chars
        sscanf(argv[i+1],"%f",&anti_w);
    }
  }

  ////////////////////////////////////
  // Error checking
  ////////////////////////////////////

  if ( rate <= 0 )
  {
    ROS_ERROR("Enter a positive value for the loop rate.");
    exit(1);
  }

  if ( ll>ul )
  {
    ROS_ERROR("The lower saturation limit cannot be greater than the upper saturation limit.");
    exit(1);
  }
  
  ////////////////////////////////////
  // Display parameters
  ////////////////////////////////////

  cout<< endl<<"PID PARAMETERS"<<endl<<"-----------------------------------------"<<endl;
  cout<<"Kp: "<<Kp<<",  Ki: "<<Ki<<",  Kd: "<<Kd<<",  Loop rate [Hz]: "<<rate<<endl;
  if ( cutoff_frequency== -1)
    cout<<"LPF cutoff frequency: 1/4 of sampling rate"<<endl;
  else
    cout<<"LPF cutoff frequency: "<<cutoff_frequency<<endl;
  cout<<"pid node name: "<<node_name<<endl;
  cout<<"Name of topic from controller: "<< topic_from_controller<<endl;
  cout<<"Name of topic from the plant: "<< topic_from_plant<<endl;
  cout<<"Anti-windup: "<<anti_w<<endl;
  cout<<"Saturation limits: "<< ul <<"/"<<ll<<endl<<"-----------------------------------------"<<endl;
}
