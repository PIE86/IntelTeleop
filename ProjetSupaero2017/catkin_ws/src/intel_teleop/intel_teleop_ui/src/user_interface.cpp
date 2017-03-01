// Include the ROS C++ APIs
#include "ros/ros.h"
//#include "std_msgs/String.h"

//#include <intel_teleop_msgs/UserInput.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <termios.h>
#include <signal.h>


// Keyboard input codes

#define KEYCODE_FORWARD  0x41 // Down key
#define KEYCODE_BACKWARD 0x42 // Up key
#define KEYCODE_LEFT     0x44 // Left key
#define KEYCODE_RIGHT    0x43 // Right key
#define KEYCODE_UP       0x7A // Z key
#define KEYCODE_DOWN     0x78 // X key


// Tranlation and rotation step

#define TRANS_STEP_KB 0.2
#define TRANS_STEP_JS 2.0
#define ROT_STEP_JS 2.0


static struct termios old_terminal, new_terminal;

geometry_msgs::Twist input;
ros::Publisher user_input_topic;
ros::Subscriber joystick_topic;
bool js;
int x_axis, y_axis, z_axis, yaw_axis;
int motor_on_button, motor_off_button;

void get_keyboard_input()
{
	// Get the next event from the keyboard
	
	int input_char;
	
	fflush(STDIN_FILENO);
	
	if(read(STDIN_FILENO, &input_char, 1) < 0)
    {
		perror("read():");
		exit(-1);
    }
    
    //ROS_INFO("%d pressed!", input_char);
    
    // Modify input message
    
    switch(input_char)
    {
      case KEYCODE_FORWARD:
        ROS_INFO("FORWARD");
        input.linear.x += TRANS_STEP_KB;
        break;
      case KEYCODE_BACKWARD:
        ROS_INFO("BACKWARD");
        input.linear.x -= TRANS_STEP_KB;
        break;
      case KEYCODE_LEFT:
        ROS_INFO("LEFT");
        input.linear.y += TRANS_STEP_KB;
        break;
      case KEYCODE_RIGHT:
        ROS_INFO("RIGHT");
        input.linear.y -= TRANS_STEP_KB;
        break;
      case KEYCODE_UP:
        ROS_INFO("UP");
        input.linear.z += TRANS_STEP_KB;
        break;
      case KEYCODE_DOWN:
        ROS_INFO("DOWN");
        input.linear.z -= TRANS_STEP_KB;
        break;
      default:
		return;
    }
    
    user_input_topic.publish(input);
    
    ROS_INFO("User input: V = [%lf,%lf,%lf]", 
			input.linear.x, input.linear.y, input.linear.z);
}


void joy_callback(const sensor_msgs::Joy joy)
{	
    input.linear.x = joy.axes[4] * TRANS_STEP_JS;
    input.linear.y = joy.axes[3] * TRANS_STEP_JS;
    input.linear.z = joy.axes[1] * TRANS_STEP_JS;
    
    input.angular.x = 0.0;
    input.angular.y = 0.0;
    input.angular.z = joy.axes[0] * ROT_STEP_JS;
    
    user_input_topic.publish(input);
    
    ROS_INFO("User input: V = [%lf,%lf,%lf]", 
			input.linear.x, input.linear.y, input.linear.z);
}


void quit(int sig)
{
	(void)sig; // Unused parameter
	tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
	ROS_INFO("Exit...");
	ros::shutdown();
	exit(0);
}


int main(int argc, char **argv)
{
	//ROS_INFO("Begin...");
	
	ros::init(argc, argv, "user_interface");
	ros::NodeHandle n;
	
	// Controller (keyboard or joystick)
	
	n.param<bool>("joystick", js, false);
        
    n.param<int>("x_axis", x_axis, 4);
    n.param<int>("y_axis", y_axis, 3);
    n.param<int>("z_axis", z_axis, 1);
    n.param<int>("yaw_axis", yaw_axis, 0);
		
    n.param<int>("motor_on_button", motor_on_button, 2);
    n.param<int>("motor_off_button", motor_on_button, 1);
	
	// Initialize user input message
		
	input.linear.x = 0.0;
	input.linear.y = 0.0;
	input.linear.z = 0.0;
	input.angular.x = 0.0;
	input.angular.y = 0.0;
	input.angular.z = 0.0;

	user_input_topic = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	if(js)
	{
		joystick_topic = n.subscribe("joy", 10, joy_callback);
	}

	signal(SIGINT,quit);

	// * Change terminal mode *

	tcgetattr(STDIN_FILENO, &old_terminal);                       // Save old settings
	memcpy(&new_terminal, &old_terminal, sizeof(struct termios)); // Initialize new settings
	new_terminal.c_lflag &= ~(ICANON | ECHO);                     // Disable buffering and typed character's echo
	new_terminal.c_cc[VEOL] = 1;
	new_terminal.c_cc[VEOF] = 2;
	new_terminal.c_cc[VMIN] = 0;                                  // Polling read
	new_terminal.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);              // Apply new settings

	ros::Rate loop_rate(100);

	loop_rate.sleep();

	while (ros::ok())
	{
		if(!js)
		{
			get_keyboard_input();
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
