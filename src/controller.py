#!/usr/bin/env python3

from pynput.mouse import Button, Controller
import qwiic_joystick
import time
import sys
import rospy
from geometry_msgs.msg import Twist, Vector3
import tkinter as tk

class Control:
    def __init__(self):
        self.joy1 = qwiic_joystick.QwiicJoystick()
        self.horizontal1 = 0.0
        self.vertical1 = 0.0
        self.button1 = False
        self.mouse = Controller()
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.timer = rospy.Time.now()
        self.mode = 1


    def updateJoystickValues(self):
        self.joy1.begin()
        self.horizontal1 = self.joy1.vertical
        self.vertical1 = self.joy1.horizontal
        self.button1 = self.joy1.button

    def controlMouse(self):
        speed = 100
        if self.joy1.connected:
            self.joy1.begin()
            if not self.joy1.button:
                print("BUTTON")
                self.mouse.click(Button.left, 1)
                time.sleep(0.5)
            self.mouse.move(self.joy1.vertical/speed-5.18, -self.joy1.horizontal/speed+5.01)


    def controlRobot(self):
        speed = 100
        if self.joy1.connected:
            omega = self.joy1.vertical/speed-5.18
            xdot = -self.joy1.horizontal/speed+5.01
            twist = Twist(linear = Vector3(x = xdot, y = 0, z = 0), angular = Vector3(x = 0, y = 0, z = omega))
            self.cmd_pub.publish(twist)

    def checkControl(self):
        if not self.joy1.button:
            if (rospy.Time.now()-self.timer).to_sec() > 2:
                if self.mode == 0:
                    self.mode = 1            
                    self.timer = rospy.Time.now()

                else:
                    self.mode = 0
                    self.timer = rospy.Time.now()

        else:
            self.timer = rospy.Time.now()


    


if __name__ == '__main__':
    rospy.init_node("controller")
    c = Control()

    while not rospy.is_shutdown():
        c.checkControl()
        if c.mode:
            c.controlMouse()
        else:
            c.controlRobot()
    rospy.spin()
    
            
