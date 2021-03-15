# ros_controller

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