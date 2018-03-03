# Launch file
To initiate the world with the car & obstacles:
roslaunch display display.launch

# Topics for command and state
Command should be applied on: /car_control/command
State is broadcast on: /car_control/state

# TODO
Obstacles are retrieved from pkg:utils/obstacles/... using the service from this package
I have to merge this with pkg:obstacles directly (more up-to-date)

