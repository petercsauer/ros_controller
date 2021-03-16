#!/usr/bin/env python3

from pynput.mouse import Button, Controller
import qwiic_joystick
import time
import sys
import rospy
from geometry_msgs.msg import Twist, Vector3
import tkinter as tk
from std_msgs.msg import String,Int32,Int32MultiArray

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
        self.buttons = rospy.get_param('/controller/buttons')
        col = []
        for i in range(len(self.buttons['id'])):
            col.append(0)
        self.button_matrix = [[1,0,0],[0,0,0],col]
        self.button_pub = rospy.Publisher('button_matrix', Int32MultiArray, queue_size=0)

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
            
    def controlButtons(self):
        if self.joy1.connected:
            self.joy1.begin()
            print(str(self.joy1.horizontal)+", "+str(self.joy1.vertical))
            if (self.joy1.horizontal<300):
                for i in range(len(self.button_matrix)):
                    for j in range(len(self.button_matrix[i])):
                        if self.button_matrix[i][j]>=1:
                            if(j>0):
                                self.button_matrix[i][j] = 0
                                self.button_matrix[i][j-1] = 1
                                time.sleep(0.5)

            elif (self.joy1.horizontal>700):
                for i in range(len(self.button_matrix)):
                    for j in range(len(self.button_matrix[i])):
                        if self.button_matrix[i][j]>=1:
                            if(j<len(self.button_matrix[i])-1):
                                self.button_matrix[i][j] = 0
                                self.button_matrix[i][j+1] = 1
                                time.sleep(0.5)
            elif (self.joy1.vertical<300):
                for i in range(len(self.button_matrix)):
                    for j in range(len(self.button_matrix[i])):
                        if self.button_matrix[i][j]>=1:
                            if(i>0):
                                self.button_matrix[i][j] = 0
                                self.button_matrix[i-1][j] = 1
                                time.sleep(0.5)
            elif (self.joy1.horizontal>700):
                for i in range(len(self.button_matrix)):
                    for j in range(len(self.button_matrix[i])):
                        if self.button_matrix[i][j]>=1:
                            if(i<len(self.button_matrix)-1 and j<len(self.button_matrix[i+1])):
                                self.button_matrix[i][j] = 0
                                self.button_matrix[i+1][j] = 1
                                time.sleep(0.5)

            if (not self.joy1.button):
                for i in range(len(self.button_matrix)):
                    for j in range(len(self.button_matrix[i])):
                        if self.button_matrix[i][j]>=1:
                            self.button_matrix[i][j] = 2
            else:
                for i in range(len(self.button_matrix)):
                    for j in range(len(self.button_matrix[i])):
                        if self.button_matrix[i][j]>=1:
                            self.button_matrix[i][j] = 1
        
        pub_array = []
        a = Int32MultiArray()
        for i in range(len(self.button_matrix)):
            for j in range(len(self.button_matrix[i])):
                pub_array.append(self.button_matrix[i][j])
            pub_array.append(100)
        a.data = pub_array
                        
        self.button_pub.publish(a)
                                


    def controlRobot(self):
        speed = 300
        if self.joy1.connected:
            omega = (self.joy1.vertical-518)/speed
            xdot = (self.joy1.horizontal-502)/speed
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
            c.controlButtons()
        else:
            c.controlRobot()
    rospy.spin()
    
            
