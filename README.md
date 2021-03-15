# ros_controller

# Usage Instructions
- Connect the robot and controller to the same network
- export ROS_HOSTNAME=http://raspberrypi.local:11311 (on robot)
- export ROS_IP=raspberrypi.local (on robot)
- roslaunch control control.launch (on controller)
- run any node to be tested on the robot

# Mouse Control Mode
- In this mode, the left joystick controls the mouse on the raspberry pi in the controller. 
- This can be useful to select functions and run programs on raspberry OS.
- Can switch between mouse control and velocity control modes by holding down the joystick button for 2 seconds.
- Video Demo: https://drive.google.com/file/d/1apWOIeSkuBDc_xTBUmET32rqAUC1X6jY/view?usp=sharing

# Velocity Control Mode
- In this mode, the left joystick sends cmd_vel commands to any connected robot
- This can be useful to drive around a diff drive style of robot.
- In this mode, movement left to right publishes to twist.angular.z and up and down movement publishes to twist.linear.x
- Video Demo: https://drive.google.com/file/d/1Vq2hrRpSAihLlSR3H_R1zQseN4hwDoo5/view?usp=sharing

# Mouse and Keyboard Control
- The controller comes with a wireless mouse and keyboard attatched, so the developer can utilize all of the raspbian os.
- In this mode, the controller becomes a full linux computer, using the eink display as a full display.
- Video Demo: https://drive.google.com/file/d/1e-eE9lNR6LbbHdcuiJWl0FAsSrEUFRB0/view?usp=sharing

# Info Modules
- Info modules have been created to allow the developer to easily display useful data from the robot.
- In this example, the robots pose, and encoder data is published here as well as some useful data about the robot.
- Video Demo: https://drive.google.com/file/d/1NOoGeiinw6MxqIVGJ60zSW3B7ncj2-ec/view?usp=sharing

# Service Modules
- Service modules have been created to allow the developer to easily test different services offered by the robot.
- Developers can click the play button next to a service option to run that service with an autocompleted command (\t).
- Developers can click the main section of the service module to input their own command to run.

