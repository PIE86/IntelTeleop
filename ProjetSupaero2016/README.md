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

Run `test_viewer` to check if gepetto installation was successful. You should see a window with a drone that you can move around with arrow keys, A, R, W, X, C, V, B and N. The red cylider shows the direction the drone is moving, for demonstration purposes. The mouse controls the camera.
- in case of "terminate called after throwing an instance of 'CORBA::TRANSIENT'", make sure gepetto-viewer-server is running in background. You also have to restart gepetto-viewer-server every time you run one of the tests.
- make sure that you have omniNames running in the background as well.

Run `test_viewer_environment` to check if tinyXML installation was successful. You should see a window with a large slanted yellow cylinder.

Run `ProjectSupaero` for the full program. You should be able to control the drone in the environment described in data/envsave.xml with arrow keys and A/R keys. The drone should avoid the obstacles. 
