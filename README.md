# Description of the project

The goal of this project is to showcase intelligent teleoperation for complex microdrones, which means adding a stabilising and obstacle avoidance loop to a human input for a quad-copter type microdrone.

The end product features:
- a simulator with a 3D viewer, which shows the drone in its obstacle riddled environment
- an MPC control loop for stabilisation and obstacle avoidance
- a simplified drone model, easily adaptable to a more complex model
- an XML parser for simple description of different environments
- a keyboard interface which can be easily modified to accept joysticks commands

# How to install

This program relies on four external libraries which you must install first:
- Gepetto viewer (https://github.com/humanoid-path-planner/gepetto-viewer and https://github.com/humanoid-path-planner/gepetto-viewer-corba)
- ACADO (http://acado.github.io/)
- SFML (http://www.sfml-dev.org/)
- tynixml2 (https://github.com/leethomason/tinyxml2)

When these libraries are properly set up, proceed as following:
```
mkdir new/project/directory/ && cd new/project/directory/
git clone https://github.com/rrazlapos/PIE-drone.git
cd PIE-drone/ProjetSupaero
mkdir build && cd build
cmake ..
make
```

The executable files are in build/tests/.

Run `test_sfml` to check if SFML installation was successful. You should see a window with a green disc pop up.

Run `test_viewer` to check if gepetto installation was successful. You should see a window with a drone that you can move around with arrow keys, A, R, W, X, C, V, B and N. The mouse controls the camera.
