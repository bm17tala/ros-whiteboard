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
    '3' : (3, 3)  # position to draw (pen down)
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

def arm_ctrl(id, direction, pub_Arm):
    global arm_joints
    global armjoint
    global keyReleased
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
    for i in range(6):
        armjoint.id = i
        if id == 1:
            armjoint.angle = pos_grab_marker[i]
        elif id == 2:
            armjoint.angle = pos_draw_hover[i]
        elif id == 3:
            armjoint.angle = pos_draw_pen_down[i]

    sleep(0.05)



    	
def Armcallback(msg):
    global arm_joints
    if not isinstance(msg, ArmJoint): return
    if len(msg.joints) != 0: arm_joints = list(msg.joints)
    else: arm_joints[msg.id - 1] = msg.angle

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_ctrl')
    #rospy.init_node('yahboom_keyboardArm')
    linear_limit = rospy.get_param('~linear_speed_limit', 1.0)
    angular_limit = rospy.get_param('~angular_speed_limit', 5.0)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #--new--
    sub_Arm = rospy.Subscriber("ArmAngleUpdate", ArmJoint, Armcallback, queue_size=1000)
    pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
    srv_arm = rospy.ServiceProxy("CurrentAngle", RobotArmArray)
    #-------
    xspeed_switch = True
    (speed, turn) = (0.05, 0.1)
    (x, th) = (0, 0)
    status = 0
    stop = False
    count = 0
    key = ''
    try:
        print(msg)
        print(vels(speed, turn))
        
        while not (key == 'h' or key == 'H'):
            # 获取当前按键信息
            key = pressedKey
            if not (key == ''): print(key)
            if key=="t" or key == "T": xspeed_switch = not xspeed_switch
            elif key == "s" or key == "S":
                print ("stop keyboard control: {}".format(not stop))
                stop = not stop
            # 按键字符串判断是否在移动字典中
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            # 按键字符串判断是否在速度字典中
            elif key in speedBindings.keys():
                #speed = speed * speedBindings[key][0]
                #turn = turn * speedBindings[key][1]
                count = 0
                # 速度限制
                if speed > linear_limit: speed = linear_limit
                if turn > angular_limit: turn = angular_limit
                print(vels(speed, turn))
                # 累计一定次数次打印一次msg信息
                if (status == 14): print(msg)
                status = (status + 1) % 15
            # 如果按键是' '或者'k'，则停止运动
            elif key == ' ': (x, th) = (0, 0)
            elif key in armBindings.keys():
                print("in armBindings")
                #srv_armcallback(srv_arm, 0)
                srv_armcallback(srv_arm)
                arm_ctrl(armBindings[key][0], armBindings[key][1], pub_Arm)
                print("arm_joints: ", arm_joints)
            else:
                # 设置如果不是长按就停止运动功能
                count = count + 1
                if count > 4: (x, th) = (0, 0)
                if (key == '\x03'): break
            # 发布消息
            twist = Twist()
            if not (key == ''): print("stop: ", stop)
            if xspeed_switch: twist.linear.x = speed * x
            else: twist.linear.y = speed * x
            twist.angular.z = turn * th
            if not stop: 
                if (not (twist.linear.x == 0)):
                    print("twist: ", twist)
                pub.publish(twist)
            sleep(0.05)
    except Exception as e: print(e)
    finally: pub.publish(Twist())
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    stopKeyboardListener()
