# Launch file
To initiate the world with the car & obstacles:
roslaunch display display.launch

# Topics for command and state
## Command
Command should be published on: /car_control/command

The command is given as a (v, omega) (floats)

## State
State is broadcasted on: /car_control/state

The state is given as a MultiArray but you can retrieve data with msg.data
msg.data is a vector [x, y, theta]

# TODO
Obstacles are retrieved from pkg:utils/obstacles/... using the service from this package
I have to merge this with pkg:obstacles directly (more up-to-date)
