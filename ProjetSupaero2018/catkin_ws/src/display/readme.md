To run the simulation:
roslaunch display gazdisplay.launch
rosrun car_model car_position_publisher.py
rosrun display gazebo_display.py

To spawn custom cylinders onto Gazebo:
rosrun display spawn_cylinder_service.py
rosrun display spawn_cylinder_client.py x_c y_c r_c        *(potentially several times)

Todo next:
retrieve obstacles from package osbtacles
