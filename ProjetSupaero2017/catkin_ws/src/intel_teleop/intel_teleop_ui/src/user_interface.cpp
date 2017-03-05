/*
 * File:        user_interface.cpp
 * Project:     intel_teleop
 * ROS node:    intel_teleop_user_interface
 * ROS package: intel_teleop_ui
 * Date:        05/03/2017
 */
 
 
// Include the ROS C++ APIs
#include "ros/ros.h"

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <termios.h>
#include <signal.h>


// Keyboard input codes

#define KEYCODE_FORWARD           0x41 // Down key
#define KEYCODE_BACKWARD          0x42 // Up key
#define KEYCODE_LEFT              0x44 // Left key
#define KEYCODE_RIGHT             0x43 // Right key
#define KEYCODE_UP                0x7A // Z key
#define KEYCODE_DOWN              0x78 // X key
#define KEYCODE_CLOCKWISE         0x39 // 9 key
#define KEYCODE_COUNTERCLOCKWISE  0x37 // 7 key
#define KEYCODE_ON                0x69 // I key
#define KEYCODE_OFF               0x6F // O key
#define KEYCODE_RESET             0x30 // 0 key


// Tranlation and rotation step

#define TRANS_STEP_KB 0.5
#define ROT_STEP_KB   0.5
#define TRANS_STEP_JS 2.0
#define ROT_STEP_JS   2.0



// Global variables

static struct termios old_terminal, new_terminal;

geometry_msgs::Twist input;
ros::Publisher user_input_topic;
ros::Subscriber joystick_topic;
ros::ServiceClient motor_enable_service;
bool js;
int x_axis, y_axis, z_axis, yaw_axis;
int motor_on_button, motor_off_button;



// Function declaration

void initialize();

void quit(int sig);

void joyCallback(const sensor_msgs::JoyConstPtr &joy);

void getKeyboardInput();

bool enableMotors(bool enable);

double getJoystickAxis(const sensor_msgs::JoyConstPtr &joy, const int &axis);
  
bool getJoystickButton(const sensor_msgs::JoyConstPtr &joy, const int &button);



// Main


int main(int argc, char **argv)
{
	ROS_INFO("Initializing UI");
	
	ros::init(argc, argv, "user_interface");
	
	initialize();	
	
	ros::Rate loop_rate(50);

	loop_rate.sleep();

	while (ros::ok())
	{
		if(!js)
		{
			getKeyboardInput();
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}



// Function implementation

void initialize()
{	
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	
	nh_param.param<bool>("joystick", js, false);
        
    nh_param.param<int>("x_axis", x_axis, 5);
    nh_param.param<int>("y_axis", y_axis, 4);
    nh_param.param<int>("z_axis", z_axis, 2);
    nh_param.param<int>("yaw_axis", yaw_axis, 1);
		
    nh_param.param<int>("motor_on_button", motor_on_button, 3);
    nh_param.param<int>("motor_off_button", motor_off_button, 2);
	
	// Initialize user input message
		
	input.linear.x = 0.0;
	input.linear.y = 0.0;
	input.linear.z = 0.0;
	input.angular.x = 0.0;
	input.angular.y = 0.0;
	input.angular.z = 0.0;

	user_input_topic = nh.advertise<geometry_msgs::Twist>("command_velocity", 10);

	if(js)
	{
		ROS_INFO("Command interface: joystick");
		
		joystick_topic = nh.subscribe("joy", 10, joyCallback);
	}
	else
	{
		ROS_INFO("Command interface: keyboard");
	}
	
	motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");

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
}


void quit(int sig)
{
	(void)sig; // Unused parameter
	
	tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
		
	if(js)
	{
		joystick_topic.shutdown();
	}
		
	ROS_INFO("Exiting UI...");
	
	ros::shutdown();
	
	exit(0);
}


void joyCallback(const sensor_msgs::JoyConstPtr &joy)
{	
    input.linear.x = getJoystickAxis(joy, x_axis) * TRANS_STEP_JS;
    input.linear.y = getJoystickAxis(joy, y_axis) * TRANS_STEP_JS;
    input.linear.z = getJoystickAxis(joy, z_axis) * TRANS_STEP_JS;
    
    input.angular.x = 0.0;
    input.angular.y = 0.0;
    input.angular.z = getJoystickAxis(joy, yaw_axis) * ROT_STEP_JS;
    
    user_input_topic.publish(input);
		
	if (getJoystickButton(joy, motor_off_button))
    {
      if(enableMotors(false))
		ROS_INFO("Disable motors");
	  else
	    ROS_WARN("Failed to disable motors");
    }
    else if (getJoystickButton(joy, motor_on_button))
    {
      if(enableMotors(true))
		ROS_INFO("Enable motors");
	  else
	    ROS_WARN("Failed to enable motors");
    }
}


void getKeyboardInput()
{
	// Get the next event from the keyboard
	
	int input_char;
	
	fflush(STDIN_FILENO);
	
	if(read(STDIN_FILENO, &input_char, 1) < 0)
    {
		perror("read():");
		exit(-1);
    }
    
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
      case KEYCODE_CLOCKWISE:
        ROS_INFO("CLOCKWISE");
        input.angular.z -= ROT_STEP_KB;
        break;
      case KEYCODE_COUNTERCLOCKWISE:
        ROS_INFO("COUNTERCLOCKWISE");
        input.angular.z += ROT_STEP_KB;
        break;
      case KEYCODE_RESET:
		ROS_INFO("RESET");
		input.linear.x = 0.0;
		input.linear.y = 0.0;
		input.linear.z = 0.0;
		input.angular.z = 0.0;
        break;
    }
    
    user_input_topic.publish(input);
			
	if (input_char == KEYCODE_OFF)
    {
      if(enableMotors(false))
		ROS_INFO("Disable motors");
	  else
	    ROS_WARN("Failed to disable motors");
    }
    else if (input_char == KEYCODE_ON)
    {
      if(enableMotors(true))
		ROS_INFO("Enable motors");
	  else
	    ROS_WARN("Failed to enable motors");
    }
}


bool enableMotors(bool enable)
{		
	if (!motor_enable_service.waitForExistence(ros::Duration(5.0)))
	{
		ROS_WARN("Motor enable service not found");
		return false;
	}

	hector_uav_msgs::EnableMotors srv;
	srv.request.enable = enable;
	
	return motor_enable_service.call(srv);
}


double getJoystickAxis(const sensor_msgs::JoyConstPtr &joy, const int &axis)
{
	if (axis == 0 || std::abs(axis) > joy->axes.size())
	{
		ROS_ERROR_STREAM("Axis " << axis << " out of range, joy has " << joy->axes.size() << " axes");
		return 0.0;
	}

	double output = std::abs(axis) / axis * joy->axes[std::abs(axis) - 1];

	return output;
}

  
bool getJoystickButton(const sensor_msgs::JoyConstPtr &joy, const int &button)
{
	if (button <= 0 || button > joy->buttons.size())
	{
		ROS_ERROR_STREAM("Button " << button << " out of range, joy has " << joy->buttons.size() << " buttons");
		return false;
	}

	return joy->buttons[button - 1] > 0;
}
