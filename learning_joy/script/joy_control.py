#!/usr/bin/env python

import time
import rospy
import pygame
import math3d as m3d
from math import pi
from geometry_msgs.msg import Twist

class Cmd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0
        self.axis3 = 0
        self.axis4 = 0
        self.axis5 = 0
        self.btn0 = 0
        self.btn1 = 0
        self.btn2 = 0
        self.btn3 = 0
        self.btn4 = 0
        self.btn5 = 0
        self.btn6 = 0
        self.btn7 = 0
        self.btn8 = 0
        self.btn9 = 0


class Service(object):
    def __init__(self, linear_velocity=0.1, rotational_velocity=0.1, acceleration=0.1):
        self.joystick = None
        self.cmd = Cmd()
        self.linear_velocity = linear_velocity
        self.rotational_velocity = rotational_velocity
        self.acceleration = acceleration
        global pub
        global msg
        msg = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def init_joystick(self):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print('Initialized Joystick : %s' % self.joystick.get_name())

    def loop(self):
        print("Starting loop")
        air = False
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd.reset()
            pygame.event.pump()#Seems we need polling in pygame...

            #get joystick state
            for i in range(0, self.joystick.get_numaxes()):
                val = self.joystick.get_axis(i)
                if i in (2, 5) and val != 0:
                    val += 1
                if abs(val) < 0.2:
                    val = 0
                tmp = "self.cmd.axis" + str(i) + " = " + str(val)
                if val != 0:
                    print(tmp)
                exec(tmp)

            #get button state
            for i in range(0, self.joystick.get_numbuttons()):
                if self.joystick.get_button(i) != 0:
                    tmp = "self.cmd.btn" + str(i) + " = 1"
                    print(tmp)
                    exec(tmp)

            #if self.cmd.btn0:
                #toggle IO
                #air = not air

            #if self.cmd.btn9:

            #initalize speed array to 0
            speeds = [0, 0, 0, 0, 0, 0]

            #get linear speed from joystick
            speeds[0] = self.cmd.axis0 * self.linear_velocity
            speeds[1] = -self.cmd.axis1 * self.linear_velocity

            if self.cmd.btn4 and not self.cmd.axis2:
                speeds[5] = -self.linear_velocity
            if self.cmd.axis2 and not self.cmd.btn4:
                speeds[5] = self.cmd.axis2 * self.linear_velocity

            #get rotational speed from joystick
            speeds[4] = -1 * self.cmd.axis3 * self.rotational_velocity
            speeds[3] = -1 * self.cmd.axis4 * self.rotational_velocity

            if self.cmd.btn5 and not self.cmd.axis5:
                speeds[5] = self.rotational_velocity
            if self.cmd.axis5 and not self.cmd.btn5:
                speeds[5] = self.cmd.axis5 * -self.rotational_velocity

            speeds = [-i for i in speeds]

            if self.cmd.btn7:
                print("WHAT????????")
            else:
                times = 5
                msg.linear.x = -1*speeds[1]*times
                msg.linear.y = speeds[0]*times
                msg.linear.z = speeds[2]*times
                msg.angular.x = speeds[3]*times
                msg.angular.y = speeds[4]*times
                msg.angular.z = speeds[5]*times
                pub.publish(msg)
            rate.sleep()

    def close(self):
        if self.joystick:
            self.joystick.quit()


if __name__ == "__main__":

    #start joystick service with given max speed and acceleration
    rospy.init_node('joy_control_node')
    service = Service(linear_velocity=0.1, rotational_velocity=0.1, acceleration=0.1)
    service.init_joystick()
    try:
        service.loop()
    finally:
        service.close()
