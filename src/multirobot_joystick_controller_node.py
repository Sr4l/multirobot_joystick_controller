#!/usr/bin/env python  

# Copyright (c) 2015, Lars Kistner
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of multirobot_joystick_controller nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# Standard Python
from __future__ import division
import math
from math import pi
from time import sleep

# ROS packages
import roslib
roslib.load_manifest('multirobot_joystick_controller')

import rospy
import tf

from kobuki_msgs.msg import MotorPower
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Config Values
REFRESH_RATE = 1.0
ROBOT_MAX_VELOCITY = 0.5
ROBOT_ROTATION_SCALE = 0.5
DEADMAN_BUTTON = 4
TWISTMSG_TOPIC = "{}/cmd_vel"
POWER_TOPIC = "{}/mobile_base/commands/motor_power"
ROBOTS_LIST = ["Turtlebot3", "Turtlebot2", "Turtlebot1", "Turtlebot4"]

class MultirobotJoystickControll(object):
    def __init__(self):
        self.joy_topic = rospy.get_param("~joy", "/joy")
        subscriber = rospy.Subscriber(self.joy_topic, Joy, self.handle_joystick)
        self.selected_robot = 0
        self.robots = ROBOTS_LIST
        self.velo_pub = [rospy.Publisher(TWISTMSG_TOPIC.format(r), Twist, queue_size=10) for r in self.robots]
        self.power_pub = [rospy.Publisher(POWER_TOPIC.format(r), MotorPower, queue_size=2) for r in self.robots]
    
    def check_new_selected_robot(self, button_states):
        robot_select = button_states[:4]
        if sum(robot_select) == 0:
            rospy.logdebug("No robot select button pressed, do nothing")
        elif sum(robot_select) == 1:
            rospy.logdebug("Robot select button pressed")
            for i, button in enumerate(robot_select):
                if button == 1:
                    rospy.loginfo("Select robot {} - {}".format(i, self.robots[i]))
                    self.selected_robot = i
        else:
            rospy.logerr("More than one button pressed, can't slelect a robot")
    
    def handle_move(self, throttle, turn):
        msg = Twist()
        msg.linear.x = throttle * ROBOT_MAX_VELOCITY
        msg.angular.z = turn*2*pi*ROBOT_ROTATION_SCALE
        
        self.velo_pub[self.selected_robot].publish(msg)

    def powerup(self, up=True):
        if up:
            rospy.loginfo("{}: powered up".format(ROBOTS_LIST[self.selected_robot]))
            self.power_pub[self.selected_robot].publish(MotorPower(1))
        else:
            rospy.loginfo("{}: powered down".format(ROBOTS_LIST[self.selected_robot]))
            self.power_pub[self.selected_robot].publish(MotorPower(0))
    
    def handle_joystick(self, msg):
        ## Xbox 360 Controller
        #print msg.axes[0:2] # keypad links
        #print msg.axes[3:5] # keypad rechts
        #print msg.axes[6:8] # keypad mitte
        #print msg.axes[2] # LT
        #print msg.axes[5] # RT
        #print msg.buttons
        
        self.check_new_selected_robot(msg.buttons)
        if msg.buttons[DEADMAN_BUTTON]:
            self.handle_move(msg.axes[1], msg.axes[3])
        else:
            self.handle_move(0.0, 0.0)
        
        if msg.axes[5] < 0:
            self.powerup(False)
        elif msg.axes[2] < 0:
            self.powerup(True)

def main():
    rospy.loginfo("Init node")
    rospy.init_node("multirobot_joystick_controller_node")
    
    rate = rospy.Rate(REFRESH_RATE)
    
    mjc = MultirobotJoystickControll()
    
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("Exit node")

if __name__ == "__main__":
    main()
