roslaunch gazebo_ros empty_world.launch&

rosrun xacro xacro Objects/cylinder.xacro x:=-5 y:=-3 radius:=1 length:=5 > object.urdf
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model model1

rosrun xacro xacro Objects/cylinder.xacro x:=0 y:=-3 radius:=1 length:=5 > object.urdf
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model model2

rosrun xacro xacro Objects/cylinder.xacro x:=5 y:=-3 radius:=1 length:=5 > object.urdf
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model model3

rosrun xacro xacro Objects/cylinder.xacro x:=-5 y:=3 radius:=1 length:=5 > object.urdf
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model model4

rosrun xacro xacro Objects/cylinder.xacro x:=0 y:=3 radius:=1 length:=5 > object.urdf
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model model5

rosrun xacro xacro Objects/cylinder.xacro x:=5 y:=3 radius:=1 length:=5 > object.urdf
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model model6

rm object.urdf

