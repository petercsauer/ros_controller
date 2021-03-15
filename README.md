# ros_controller

## Usage Instructions
- Connect the robot and controller to the same network
- export ROS_HOSTNAME=http://raspberrypi.local:11311 (on robot)
- export ROS_IP=raspberrypi.local (on robot)
- roslaunch control control.launch (on controller)
- run any node to be tested on the robot

## Software
### Mouse Control Mode
- In this mode, the left joystick controls the mouse on the raspberry pi in the controller. 
- This can be useful to select functions and run programs on raspberry OS.
- Can switch between mouse control and velocity control modes by holding down the joystick button for 2 seconds.
- Video Demo: https://drive.google.com/file/d/1apWOIeSkuBDc_xTBUmET32rqAUC1X6jY/view?usp=sharing

### Velocity Control Mode
- In this mode, the left joystick sends cmd_vel commands to any connected robot
- This can be useful to drive around a diff drive style of robot.
- In this mode, movement left to right publishes to twist.angular.z and up and down movement publishes to twist.linear.x
- Video Demo: https://drive.google.com/file/d/1Vq2hrRpSAihLlSR3H_R1zQseN4hwDoo5/view?usp=sharing

### Mouse and Keyboard Control
- The controller comes with a wireless mouse and keyboard attatched, so the developer can utilize all of the raspbian os.
- In this mode, the controller becomes a full linux computer, using the eink display as a full display.
- Video Demo: https://drive.google.com/file/d/1e-eE9lNR6LbbHdcuiJWl0FAsSrEUFRB0/view?usp=sharing

### Info Modules
- Info modules have been created to allow the developer to easily display useful data from the robot.
- In this example, the robots pose, and encoder data is published here as well as some useful data about the robot.
- Video Demo: https://drive.google.com/file/d/1NOoGeiinw6MxqIVGJ60zSW3B7ncj2-ec/view?usp=sharing

### Service Modules
- Service modules have been created to allow the developer to easily test different services offered by the robot.
- Developers can click the play button next to a service option to run that service with an autocompleted command (\t).
- Developers can click the main section of the service module to input their own command to run.

## Hardware
### BOM
##### Electronics
- Microcontroller: RaspberryPI 4 (4GB RAM) (
- Display Driver: Waveshare HDMI Interface (https://www.waveshare.com/7.8inch-hdmi-e-paper.htm)
- Joysticks: Sparkfun Qwiic Joystick (https://www.sparkfun.com/products/15168)
- Display: Waveshare 7.8" Eink Display (https://www.waveshare.com/7.8inch-hdmi-e-paper.htm)

##### Cables
- USB C Extension: https://www.digikey.com/en/products/detail/sparkfun-electronics/CAB-15455/10819694
- HDMI (Mini to Micro): https://www.amazon.com/Nanosecond-Extreme-Slim-Mini-Cable/dp/B01NC2X1DM/ref=sr_1_5?dchild=1&keywords=mini+to+micro+hdmi&qid=1615850285&sr=8-5
- Raspberry PI Power Supply: https://vilros.com/collections/official-raspberry-pi-accessories/products/official-raspberry-pi-foundation-power-supply-for-raspberry-pi-4-us-white-ul?src=raspberrypi 

##### Construction Materials
- Black 1/4" Acrylic (Laser Cut)
- 2" 10-32 Machine Screws (Black Oxide) (4X)
- 10-32 Machine Nuts (4x)
- Double Sided 3M Foam Tape

### CAD
##### CAD Files
- The CAD files for this project are located in the Hardware directory of this repository
- Hardware/Assemblies/FullAssembly.sldasm: The Full controller assembly
- Hardware/Assemblies/FrontAssembly.sldasm: The assembly of the front screen and joystick controllers
- Hardware/Assemblies/BackAssembly.sldasm: The assembly of the RPi4 and HDMI Interface boards
- Hardware/DXF: The DXF files for the different laser cut panels that make up the casing

##### Renders
![alt text](./Hardware/Renders/FullAssembly.jpg "Full Assembly")





