#!/usr/bin/env python
# encoding: utf-8
import os
import time
import rospy
import threading
from time import sleep
from sensor_msgs.msg import Joy
from yahboomcar_msgs.msg import *
from yahboomcar_msgs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from actionlib_msgs.msg import GoalID


class JoyTeleop:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.Joy_active = False
        self.Buzzer_active = False
        self.gripper_active = True
        self.getArm_active = True
        self.loop_active = True
        self.RGBLight_index = 0
        self.geta_arm_index = 0
        self.cancel_time = time.time()
        self.linear_Gear = 1
        self.angular_Gear = 1
        self.rate = rospy.Rate(20)
        self.armjoint = ArmJoint()
        self.armjoint.run_time = 10
        self.arm_joints = [90, 90, 90, 90, 90, 90]
        self.xspeed_limit = rospy.get_param('~xspeed_limit', 1.0)
        self.yspeed_limit = rospy.get_param('~yspeed_limit', 1.0)
        self.angular_speed_limit = rospy.get_param('~angular_speed_limit', 5.0)
        self.pub_goal = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
        self.pub_cmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_Buzzer = rospy.Publisher("Buzzer", Bool, queue_size=1)
        self.pub_JoyState = rospy.Publisher("JoyState", Bool, queue_size=10)
        self.pub_RGBLight = rospy.Publisher("RGBLight", Int32, queue_size=10)
        self.pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
        self.sub_Joy = rospy.Subscriber('joy', Joy, self.buttonCallback)
        self.sub_Arm = rospy.Subscriber("ArmAngleUpdate", ArmJoint, self.Armcallback, queue_size=1000)
        self.srv_arm = rospy.ServiceProxy("CurrentAngle", RobotArmArray)

    def srv_armcallback(self):
        self.getArm_active = False
        self.srv_arm.wait_for_service()
        request = RobotArmArrayRequest()
        request.apply = "GetArmJoints"
        try:
            response = self.srv_arm.call(request)
            if isinstance(response, RobotArmArrayResponse):
                # print ("response: ", response.angles)
                for i in range(len(response.angles)):
                    if response.angles[i] == -1:
                        if self.geta_arm_index <= 10:
                            self.geta_arm_index += 0
                            self.srv_armcallback()
                        else: self.geta_arm_index = 0
                    else:
                        self.arm_joints[i] = response.angles[i]
                print ("arm_joints: ", self.arm_joints)
        except Exception: rospy.loginfo("arg error")

    def pub_armjoint(self, id, direction):
        self.loop_active = True
        arm_thread = threading.Thread(target=self.arm_ctrl, args=(id, direction))
        arm_thread.setDaemon(True)
        arm_thread.start()

    def Armcallback(self, msg):
        if not isinstance(msg, ArmJoint): return
        if len(msg.joints) != 0: self.arm_joints = list(msg.joints)
        else: self.arm_joints[msg.id - 1] = msg.angle

    def arm_ctrl(self, id, direction):
        while 1:
            if self.loop_active:
                self.arm_joints[id - 1] += direction
                if id == 5:
                    if self.arm_joints[id - 1] > 270: self.arm_joints[id - 1] = 270
                    elif self.arm_joints[id - 1] < 0: self.arm_joints[id - 1] = 0
                elif id == 6:
                    if self.arm_joints[id - 1] >= 180: self.arm_joints[id - 1] = 180
                    elif self.arm_joints[id - 1] <= 30: self.arm_joints[id - 1] = 30
                else:
                    if self.arm_joints[id - 1] > 180: self.arm_joints[id - 1] = 180
                    elif self.arm_joints[id - 1] < 0: self.arm_joints[id - 1] = 0
                self.armjoint.id = id
                self.armjoint.angle = self.arm_joints[id - 1]
                # print "id: {},direction: {}".format(id, direction)
                self.pub_Arm.publish(self.armjoint)
            else: break
            sleep(0.03)

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return
        if self.getArm_active: self.srv_armcallback()
        # print ("joy_data.axes: ", joy_data.axes)
        # print ("joy_data.buttons: ", joy_data.buttons)
        if len(joy_data.buttons) == 15: self.user_jetson(joy_data)
        else: self.user_pc(joy_data)
    def user_jetson(self, joy_data):
        '''
        :jetson joy_data:
            axes 8: [0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0]
            左摇杆(左正右负): axes[0]
            左摇杆(上正下负): axes[1]
            右摇杆(左正右负): axes[2]
            右摇杆(上正下负): axes[3]
            R2(按负抬正): axes[4]
            L2(按负抬正): axes[5]
            左按键(左正右负): axes[6]
            左按键(上正下负): axes[7]
            buttons 15:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            A: buttons[0]
            B: buttons[1]
            X: buttons[3]
            Y: buttons[4]
            L1: buttons[6]
            R1: buttons[7]
            SELECT: buttons[10]
            START: buttons[11]
            左摇杆按下: buttons[13]
            右摇杆按下: buttons[14]
        '''
        if joy_data.buttons[10] == 1: self.gripper_active = not self.gripper_active
        if joy_data.buttons[0] == joy_data.buttons[1] == joy_data.buttons[
            6] == joy_data.buttons[3] == joy_data.buttons[4] == 0 and joy_data.axes[
            7] == joy_data.axes[6] == 0 and joy_data.axes[5] != -1: self.loop_active = False
        else:
            if joy_data.buttons[3] == 1: self.pub_armjoint(1, -1) # X
            if joy_data.buttons[1] == 1: self.pub_armjoint(1, 1)  # B
            if joy_data.buttons[0] == 1: self.pub_armjoint(2, -1) # A
            if joy_data.buttons[4] == 1: self.pub_armjoint(2, 1)  # Y
            if joy_data.axes[6] != 0: self.pub_armjoint(3, -joy_data.axes[6]) # 左按键左正右负
            if joy_data.axes[7] != 0: self.pub_armjoint(4, joy_data.axes[7]) # 左按键上正下负
            if self.gripper_active:
                if joy_data.axes[5] == -1: self.pub_armjoint(6, -1)  # L2
                if joy_data.buttons[6] == 1: self.pub_armjoint(6, 1) # L1
            else:
                if joy_data.axes[5] == -1: self.pub_armjoint(5, -1)  # L2
                if joy_data.buttons[6] == 1: self.pub_armjoint(5, 1) # L1
        # Cancel
        if joy_data.axes[4] == -1: self.cancel_nav()
        # RGBLight
        if joy_data.buttons[7] == 1:
            if self.RGBLight_index < 6: self.pub_RGBLight.publish(self.RGBLight_index)
            else: self.RGBLight_index = 0
            self.RGBLight_index += 1
        # Buzze
        if joy_data.buttons[11] == 1:
            self.Buzzer_active = not self.Buzzer_active
            self.pub_Buzzer.publish(self.Buzzer_active)
        # linear Gear control
        if joy_data.buttons[13] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        # angular Gear control
        if joy_data.buttons[14] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear
        if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        for i in range(3): self.pub_cmdVel.publish(twist)

    def user_pc(self, joy_data):
        '''
        :pc joy_data:
            axes 8: [ -0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0 ]
            左摇杆左正右负: axes[0]
            左摇杆上正下负: axes[1]
            L2按负抬正:  axes[2]
            右摇杆左正右负: axes[3]
            右摇杆上正下负: axes[4]
            R2按负抬正:  axes[5]
            左按键左正右负: axes[6]
            左按键上正下负: axes[7]
            buttons 11: [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
            A: buttons[0]
            B: buttons[1]
            X: buttons[2]
            Y: buttons[3]
            L1: buttons[4]
            R1: buttons[5]
            SELECT: buttons[6]
            MODE: buttons[7]
            START: buttons[8]
            左摇杆按下: buttons[9]
            右摇杆按下: buttons[10]
        '''
        # print ("joy_data.buttons: ", joy_data.buttons)
        # print ("joy_data.axes: ", joy_data.axes)
        if joy_data.buttons[6] == 1: self.gripper_active = not self.gripper_active
        if joy_data.buttons[0] == joy_data.buttons[1] == joy_data.buttons[
            2] == joy_data.buttons[3] == joy_data.buttons[4] == 0 and joy_data.axes[
            7] == joy_data.axes[6] == 0 and joy_data.axes[2] != -1: self.loop_active = False
        else:
            if joy_data.buttons[2] == 1: self.pub_armjoint(1, -1) # X
            if joy_data.buttons[1] == 1: self.pub_armjoint(1, 1)  # B
            if joy_data.buttons[0] == 1: self.pub_armjoint(2, -1) # A
            if joy_data.buttons[3] == 1: self.pub_armjoint(2, 1)  # Y
            if joy_data.axes[6] != 0: self.pub_armjoint(3, -joy_data.axes[6]) # 左按键左正右负
            if joy_data.axes[7] != 0: self.pub_armjoint(4, joy_data.axes[7]) # 左按键上正下负
            if self.gripper_active:
                if joy_data.axes[2] == -1: self.pub_armjoint(6, -1)  # L2
                if joy_data.buttons[4] == 1: self.pub_armjoint(6, 1) # L1
            else:
                if joy_data.axes[2] == -1: self.pub_armjoint(5, -1)  # L2
                if joy_data.buttons[4] == 1: self.pub_armjoint(5, 1) # L1
        # 取消 Cancel
        if joy_data.axes[5] == -1: self.cancel_nav()
        if joy_data.buttons[5] == 1:
            if self.RGBLight_index < 6: self.pub_RGBLight.publish(self.RGBLight_index)
            else: self.RGBLight_index = 0
            self.RGBLight_index += 1
        if joy_data.buttons[7] == 1:
            self.Buzzer_active = not self.Buzzer_active
            self.pub_Buzzer.publish(self.Buzzer_active)
        # 档位控制 Gear control
        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[3]) * self.angular_speed_limit * self.angular_Gear
        if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        for i in range(3): self.pub_cmdVel.publish(twist)

    def filter_data(self, value):
        if abs(value) < 0.2: value = 0
        return value

    def cancel_nav(self):
        self.geta_arm_index = 0
        self.getArm_active = True
        # 发布move_base取消命令
        # Issue the move_base cancel command
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.Joy_active = not self.Joy_active
            for i in range(3):
                self.pub_JoyState.publish(Bool(self.Joy_active))
                self.pub_Buzzer.publish(False)
                self.pub_goal.publish(GoalID())
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = now_time

    def cancel(self):
        self.pub_cmdVel.unregister()
        self.pub_goal.unregister()
        self.pub_JoyState.unregister()
        self.sub_Joy.unregister()


if __name__ == '__main__':
    rospy.init_node('joy_ctrl')
    joy = JoyTeleop()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
