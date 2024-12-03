#!/usr/bin/env python
# encoding: utf-8
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import time
import rospy
from time import sleep
import threading
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from yahboomcar_msgs.msg import *
from yahboomcar_msgs.srv import *
from std_msgs.msg import Int32, Bool
from actionlib_msgs.msg import GoalID
from autopilot_common import *
from pynput import keyboard

msg = """
Control Your SLAM-Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

this is to test that the correct file is working
   
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}

speedBindings = {
    'Q': (1.01, 1.01),
    'Z': (.99, .99),
    'W': (1.1, 1),
    'X': (.9, 1),
    'E': (1, 1.1),
    'C': (1, .9),
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

# armBindings = {
#     '1': (1, -1), #left
#     '2': (1, 1),  #right
#     '3': (2, 1),  #up
#     '4': (2, -1), #down
#     '5': (3, 1),  #up
#     '6': (3, -1), #down
#     '7': (4, 1),  #up
#     '8': (4, -1), #down
#     '9': (5, -1), #left
#     '0': (5, 1),  #right
#     '-': (6, 1),  #close
#     '=': (6, -1), #open
# }

armBindings = {
    '1' : (1, 1), # position to grab marker
    '2' : (2, 2), # position to draw (hover)
    '3' : (3, 3), # position to draw (pen down)
    '-' : (4, 4), # close the pinchers
    '=' : (5, 5)  # open the pinchers
}

pos_grab_marker = [90, 0, 90, 90, 90, 90]
pos_draw_hover = [90, 81, 0, 72, 88, 156]
pos_draw_pen_down = [90, 46, 0, 72, 88, 156]


arm_joints = [90, 90, 90, 90, 90, 90]
armjoint = ArmJoint()
armjoint.run_time = 10

def srv_armcallback(srv_arm):
        global arm_joints
        srv_arm.wait_for_service()
        request = RobotArmArrayRequest()
        request.apply = "GetArmJoints"
        try:
            response = srv_arm.call(request)
            print("done service")
            if isinstance(response, RobotArmArrayResponse):
                print ("response: ", response.angles)
                for i in range(len(response.angles)):
                    if response.angles[i] == -1:
                        if geta_arm_index <= 10:
                            geta_arm_index += 0
                            srv_armcallback()
                        else: geta_arm_index = 0
                    else:
                        arm_joints[i] = response.angles[i]
                print ("arm_joints: ", arm_joints)
        except: 
            rospy.loginfo("arg error")
            #try again
            #srv_armcallback(srv_arm)
            
#The value of the pressed key
pressedKey = ''
#Whether a key is released after it's pushed
keyReleased = True

#log key pressed for the keys that control arms
def on_key_press(key):
    global keyReleased
    global pressedKey
    print('Pressed Key %s' % key)
    try:
        pressedKey = key.char
        
        if key.char in armBindings.keys():
            keyReleased = False
            
    except:
        #do nothing else
        print("key is not a char: ", key)
        
#log key released for the keys that control arms
def on_key_release(key):
    global keyReleased
    global pressedKey
    print('Released Key %s' % key)
    
    try:
        pressedKey = ''
        
        if key.char in armBindings.keys():
            keyReleased = True
    except:
        #do nothing else
        print("key is not a char: ", key)
        
listener = keyboard.Listener(on_release = on_key_release, on_press=on_key_press)
listener.start()

def stopKeyboardListener():
    global listener
    listener.stop()
    os.system('clear')
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)
        
def getKey():
    # tty.setraw():将文件描述符fd模式更改为raw；fileno():返回一个整形的文件描述符(fd)
    tty.setraw(sys.stdin.fileno())
    # select():直接调用操作系统的IO接口；监控所有带fileno()方法的文件句柄
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    # 读取一个字节的输入流
    if rlist: key = sys.stdin.read(1)
    else: key = ''
    # tcsetattr从属性设置文件描述符fd的tty属性
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


ros_ctrl = ROSCtrl()
def arm_ctrl(id, direction):
    global arm_joints
    global armjoint
    global keyReleased
    global ros_ctrl
    print("id, direction: ", id, direction)
    print("key released: ", keyReleased)
    idIndex = id - 1
    # while not keyReleased:
    #     arm_joints[idIndex] += direction
    #     if id == 5:
    #         if arm_joints[idIndex] > 270: arm_joints[idIndex] = 270
    #         elif arm_joints[idIndex] < 0: arm_joints[idIndex] = 0
    #     elif id == 6:
    #         if arm_joints[idIndex] >= 180: arm_joints[idIndex] = 180
    #         elif arm_joints[idIndex] <= 30: arm_joints[idIndex] = 30
    #     else:
    #         if arm_joints[idIndex] > 180: arm_joints[idIndex] = 180
    #         elif arm_joints[idIndex] < 0: arm_joints[idIndex] = 0
    #     armjoint.id = id
    #     armjoint.angle = arm_joints[idIndex]
    #     # print "id: {},direction: {}".format(id, direction)
    #     pub_Arm.publish(armjoint)
    #     print("---------------")
    #     for i in range(len(arm_joints)):
    #         print("Joint ", i + 1, " : ", arm_joints[i], " ")
    #     print("---------------")
    #     sleep(0.05)


    # for i in range(6):
    #     armjoint.id = i + 1

    #     if id == 1:
    #         armjoint.angle = pos_grab_marker[i]
    #     elif id == 2:
    #         armjoint.angle = pos_draw_hover[i]
    #     elif id == 3:
    #         armjoint.angle = pos_draw_pen_down[i]
        
    #     sleep(0.25)
    #     pub_Arm.publish(armjoint)

    if id == 1 or id == 2 or id == 3: 
        for i in range(5):
            joint_num = i + 1

            if id == 1: # position to grab marker
                ros_ctrl.pubArm([], id=joint_num, angle=pos_grab_marker[i], run_time=1000)
            elif id == 2: # position to hover over board (pen up)
                ros_ctrl.pubArm([], id=joint_num, angle=pos_draw_hover[i], run_time=1000)
            elif id == 3: # position to draw on board (pen down)
                ros_ctrl.pubArm([], id=joint_num, angle=pos_draw_pen_down[i], run_time=1000)
            
            sleep(0.25)

    else:
        if id == 4: # close the pinchers
            ros_ctrl.pubArm([], id=6, angle=156, run_time=5000)
        elif id == 5: # open the pinchers
            ros_ctrl.pubArm([], id=6, angle=90, run_time=5000)


    sleep(0.05)



    	
def Armcallback(msg):
    global arm_joints
    if not isinstance(msg, ArmJoint): return
    if len(msg.joints) != 0: arm_joints = list(msg.joints)
    else: arm_joints[msg.id - 1] = msg.angle

pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)


