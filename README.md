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

# Velocity Control Mode
- In this mode, the left joystick sends cmd_vel commands to any connected robot
- This can be useful to drive around a diff drive style of robot.
- In this mode, movement left to right publishes to twist.angular.z and up and down movement publishes to twist.linear.x

# Info Modules
- Info modules have been created to allow the developer to easily display useful data from the robot.
- In this example, the robots pose, and encoder data is published here as well as some useful data about the robot.

# Service Modules
- Service modules have been created to allow the developer to easily test different services offered by the robot.
- Developers can click the play button next to a service option to run that service with an autocompleted command (\t).
- Developers can click the main section of the service module to input their own command to run.

