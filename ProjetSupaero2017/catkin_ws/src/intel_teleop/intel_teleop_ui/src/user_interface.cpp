// Include the ROS C++ APIs
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <intel_teleop_msgs/UserInput.h>
#include <sensor_msgs/Joy.h>
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

#define TRANS_STEP 0.1
//#define ROT_STEP 0.1


static struct termios old_terminal, new_terminal;

intel_teleop_msgs::UserInput input;

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
        input.x += TRANS_STEP;
        break;
      case KEYCODE_BACKWARD:
        ROS_INFO("BACKWARD");
        input.x -= TRANS_STEP;
        break;
      case KEYCODE_LEFT:
        ROS_INFO("LEFT");
        input.y += TRANS_STEP;
        break;
      case KEYCODE_RIGHT:
        ROS_INFO("RIGHT");
        input.y -= TRANS_STEP;
        break;
      case KEYCODE_UP:
        ROS_INFO("UP");
        input.z += TRANS_STEP;
        break;
      case KEYCODE_DOWN:
        ROS_INFO("DOWN");
        input.z -= TRANS_STEP;
        break;
    }    
}


void joy_callback(const sensor_msgs::Joy joy)
{	
    input.x = joy.axes[0] * TRANS_STEP;
    input.y = joy.axes[1] * TRANS_STEP;
    input.z = joy.axes[2] * TRANS_STEP;
    input.roll = 0.0;
    input.pitch = 0.0;
    input.yaw = 0.0;
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
	ros::init(argc, argv, "user_interface");
	ros::NodeHandle n;
	
	// Initialize user input message
	
	input.x = 0.0;
	input.y = 0.0;
	input.z = 0.0;
	input.roll = 0.0;
	input.pitch = 0.0;
	input.yaw = 0.0;

	ros::Publisher user_input_topic = n.advertise<intel_teleop_msgs::UserInput>("user_input", 1000);

	ros::Subscriber joystick_topic = n.subscribe("joy", 1000, joy_callback);

	signal(SIGINT,quit);

	/* Change terminal mode */

	tcgetattr(STDIN_FILENO, &old_terminal);                       // Save old settings
	memcpy(&new_terminal, &old_terminal, sizeof(struct termios)); // Initialize new settings
	new_terminal.c_lflag &= ~(ICANON | ECHO);                     // Disable buffering and typed character's echo
	new_terminal.c_cc[VEOL] = 1;
	new_terminal.c_cc[VEOF] = 2;
	new_terminal.c_cc[VMIN] = 0;                                  // Polling read
	new_terminal.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);              // Apply new settings

	ros::Rate loop_rate(10);

	loop_rate.sleep();

	while (ros::ok())
	{
		get_keyboard_input();

		ROS_INFO("User input: [%lf,%lf,%lf] ; [%lf,%lf,%lf]", 
			input.x, input.y, input.z, input.roll, input.pitch, input.yaw);

		user_input_topic.publish(input);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
