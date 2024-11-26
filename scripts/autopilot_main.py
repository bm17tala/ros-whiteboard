#!/usr/bin/env python
# encoding: utf-8
import os
import threading
from time import sleep
from autopilot_common import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from arm_autopilot.cfg import AutoPilotPIDConfig


class LineDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("LineDetect", anonymous=False)
        self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        self.hsv_yaml = HSVYaml()
        self.dyn_update = False
        self.Calibration = False
        self.select_flags = False
        self.gripper_state = False
        self.location_state = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        self.color.target_color_name = 'red'
        self.color_name_list = ['red', 'green', 'blue', 'yellow']
        self.hsv_value = ()
        self.color_cfg_src = self.index = self.cols = self.rows = 0
        self.Mouse_XY = (0, 0)
        self.Roi_init = ()
        Server(AutoPilotPIDConfig, self.dyn_cfg_callback)
        self.dyn_client = Client("LineDetect", timeout=60)
        self.scale = 1000.0
        self.FollowLinePID = (30.0, 0.0, 60.0)
        self.linear = 0.10 # 速度太快会导致机器人在移动的过程中看不到障碍物
        self.PID_init()
        self.joints_init = [90, 120, 0, 0, 90, 30]
        for i in range(4):
            self.color.color_hsv_list[self.color_name_list[i]] = self.hsv_yaml.read_hsv(self.color_name_list[i])
        cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback(self.windows_name, self.onMouse, 0)
        self.ros_ctrl.pubArm(self.joints_init)

    def process(self, rgb_img, action):
        if action == 32 or self.ros_ctrl.joy_action==2:
            self.Track_state = 'tracking'
            self.Calibration = False
            self.dyn_update = True
            self.ros_ctrl.pubArm(self.joints_init)
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('q') or action == ord('Q'): self.cancel()
        elif action == ord('c') or action == ord('C'):
            self.Calibration = not self.Calibration
            self.dyn_update = True
        elif action == ord('i') or action == ord('I'):
            self.Track_state = "identify"
            self.Calibration = False
            self.dyn_update = True
        elif action == ord('f') or action == ord('F'):
            color_index = self.color_name_list.index(self.color.target_color_name)
            if color_index >= 3: color_index = 0
            else: color_index += 1
            self.color.target_color_name = self.color_name_list[color_index]
            self.hsv_value = self.hsv_yaml.read_hsv(self.color.target_color_name)
            self.dyn_update = True
        if self.Track_state == 'init':
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_value = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.color.color_hsv_list[self.color.target_color_name] = self.hsv_value
                    self.hsv_yaml.write_hsv(self.color.target_color_name, self.hsv_value)
                    self.dyn_update = True
                else: self.Track_state = 'init'
        if self.Track_state != 'init' and len(self.hsv_value) != 0:
            if self.Calibration:
                self.color.msg_box = {}
                self.color.line_follow(rgb_img, self.color.target_color_name, self.hsv_value)
            else:
                for i in range(len(self.color_name_list)):
                    threading.Thread(target=self.color.line_follow,
                                     args=(rgb_img, self.color_name_list[i],
                                           self.color.color_hsv_list[self.color_name_list[i]],)).start()
        if self.Track_state == 'tracking' and len(self.color.msg_circle) != 0 and \
                not self.ros_ctrl.Joy_active and not self.gripper_state:
            for i in self.color.msg_circle.keys():
                if i == self.color.target_color_name and not self.location_state:
                    threading.Thread(target=self.execute,
                                     args=(self.color.msg_circle[self.color.target_color_name],)).start()
            for i in self.color.msg_box.keys():
                if i != self.color.target_color_name and len(self.color.msg_box) != 0 and len(self.color.msg_box[i]) != 0:
                    (point_x, point_y), _ = cv.minEnclosingCircle(self.color.msg_box[i])
                    threading.Thread(target=self.Wrecker, args=(point_x, point_y,)).start()
                else:
                    self.index += 1
                    if self.index >= 20: self.location_state = False
        else:
            if self.ros_ctrl.RobotRun_status == True: self.ros_ctrl.pubVel(0, 0)
        if self.dyn_update == True: self.dyn_cfg_update()
        return self.color.binary

    def Wrecker(self, point_x, point_y):
        self.index = 0
        self.location_state = True
        if self.ros_ctrl.Buzzer_state == True: self.ros_ctrl.pubBuzzer(False)
        if abs(point_x - 320) < 40: point_x = 320 #10
        if abs(point_y - 400) < 40: point_y = 400
        #这里开始下爪抓取
        if abs(point_x - 320) < 10 and abs(point_y - 400) < 10: #10 20
            #sleep(0.3)
            if self.ros_ctrl.RobotRun_status == True: self.ros_ctrl.pubVel(0, 0)
            self.gripper_state = True
            sleep(0.3)
            np_array = np.array([linear([320, 90], [343.5, 95])])
            pos1 = np.dot(np_array, np.array([point_x, 1])).squeeze().tolist()
            joints = [pos1, 7.0, 60.0, 38.0, 90]
            # joints = [pos1, 8.0, 49.0, 46.0, 90]
            # print ("joints: ", joints, type(joints))
            if len(joints) != 0: 
                #print(point_x)
                #print(point_y)
                self.arm_gripper(joints)
            self.color.msg_box = {}
            self.color.msg_circle = {}
            sleep(2.5)
            self.gripper_state = False
            self.location_state = False
        else: self.robot_location(point_x, point_y)

    def robot_location(self, point_x, point_y):
        [y, x] = self.PID_controller.update([(point_x - 320) / 10.0, (point_y - 400) / 10.0])
        # print ("point_x: {}, point_y: {}".format(point_x, point_y))
        #print ("x: {},y: {}".format(x, y))#15
        if x >= 0.10: x = 0.10 #0.10
        elif x <= -0.10: x = -0.10
        if y >= 0.10: y = 0.10
        elif y <= -0.10: y = -0.10
        self.ros_ctrl.pubVel(x, y)
        self.ros_ctrl.RobotRun_status = True

    def arm_gripper(self, joints):
        joints.append(30)
        self.ros_ctrl.pubArm(joints, run_time=8000)
        sleep(2)
        self.ros_ctrl.pubArm([], id=6, angle=150)
        sleep(0.5)
        self.ros_ctrl.pubArm([], id=2, angle=60, run_time=1000)
        sleep(1)
        self.ros_ctrl.pubArm([], id=1, angle=0, run_time=1000)
        sleep(1)
        joints[0] = 0
        joints[5] = 140
        self.ros_ctrl.pubArm(joints, run_time=1000)
        sleep(1)
        self.ros_ctrl.pubArm([], id=6, angle=30)
        sleep(0.5)
        self.ros_ctrl.pubArm([], id=2, angle=90, run_time=1000)
        sleep(1)
        self.ros_ctrl.pubArm([90, 120, 0, 0, 90, 30], run_time=2000)

    def execute(self, circle):
        self.index = 0
        if len(circle) == 0: self.ros_ctrl.pubVel(0, 0)
        else:
            if self.ros_ctrl.warning > 10:
                rospy.loginfo("Obstacles ahead !!!")
                self.ros_ctrl.pubVel(0, 0)
                self.ros_ctrl.pubBuzzer(True)
                self.ros_ctrl.Buzzer_state = True
            else:
                [z_Pid, _] = self.PID_controller.update([(circle[0] - 320) / 16, 0])
                if self.ros_ctrl.img_flip == True: z = -z_Pid
                else: z = z_Pid
                x = self.linear
                if self.ros_ctrl.Buzzer_state == True: self.ros_ctrl.pubBuzzer(False)
                self.ros_ctrl.pubVel(x, 0, z=z)
                # rospy.loginfo("point_x: {},linear: {}, z_Pid: {}".format(circle[0], x, z))
            self.ros_ctrl.RobotRun_status = True

    def dyn_cfg_update(self):
        hsv = self.color.color_hsv_list[self.color.target_color_name]
        params = {'Calibration': self.Calibration,
                  'Color': self.color.target_color_name,
                  'Hmin': hsv[0][0], 'Hmax': hsv[1][0],
                  'Smin': hsv[0][1], 'Smax': hsv[1][1],
                  'Vmin': hsv[0][2], 'Vmax': hsv[1][2]}
        self.dyn_client.update_configuration(params)
        self.dyn_update = False

    def dyn_cfg_callback(self, config, level):
        self.scale = config['scale']
        self.linear = config['linear']
        self.ros_ctrl.LaserAngle = config['LaserAngle']
        self.ros_ctrl.ResponseDist = config['ResponseDist']
        self.FollowLinePID = (config['Kp'], config['Ki'], config['Kd'])
        if self.Track_state != 'mouse':
            self.hsv_value = (
                (config['Hmin'], config['Smin'], config['Vmin']),
                (config['Hmax'], config['Smax'], config['Vmax']))
        else:self.Track_state = 'identify'
        self.Calibration = config["Calibration"]
        color_cfg = config["Color"]
        if self.color_cfg_src != color_cfg:
            self.hsv_value = self.hsv_yaml.read_hsv(self.color.target_color_name)
            # print ("self.color_cfg_src: {},color_cfg: {}".format(self.color_cfg_src, color_cfg))
            self.dyn_update = True
            self.color_cfg_src = color_cfg
        else:
            self.hsv_yaml.write_hsv(self.color.target_color_name, self.hsv_value)
        self.color.color_hsv_list[self.color.target_color_name] = self.hsv_value
        self.color.target_color_name = self.color_name_list[color_cfg]
        self.PID_init()
        return config

    def putText_img(self, frame):
        if self.Calibration: cv.putText(frame, "Calibration", (500, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        cv.putText(frame, self.color.target_color_name, (300, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        msg_index = len(self.color.msg_box.keys())
        if msg_index != 0:
            for i in self.color.msg_box.keys():
                try: self.color.add_box(i)
                except Exception as e: print ("e: ", e)
        self.ros_ctrl.pubImg(frame)
        return frame

    def onMouse(self, event, x, y, flags, param):
        if x > 640 or y > 480: return
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Calibration  = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def Reset(self):
        self.PID_init()
        self.color.binary = ()
        self.color.msg_box = {}
        self.Track_state = 'init'
        self.color.msg_circle = {}
        self.gripper_state = False
        self.ros_ctrl.Joy_active = False
        self.Mouse_XY = (0, 0)
        self.ros_ctrl.pubVel(0, 0)
        self.ros_ctrl.pubBuzzer(False)
        rospy.loginfo("Reset succes!!!")

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / (self.scale), self.FollowLinePID[0] / (self.scale)],
            [self.FollowLinePID[1] / (self.scale), self.FollowLinePID[1] / (self.scale)],
            [self.FollowLinePID[2] / (self.scale), self.FollowLinePID[2] / (self.scale)])

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        print("Shutting down this node.")


if __name__ == '__main__':
    line_detect = LineDetect()
    capture = cv.VideoCapture("/dev/usb_cam")
    cv_edition = cv.__version__
    if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        action = cv.waitKey(10) & 0xFF
        if line_detect.ros_ctrl.img_flip == True: frame = cv.flip(frame, 1)
        line_detect.color.frame = frame
        binary = line_detect.process(frame, action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        frame = line_detect.putText_img(frame)
        if len(binary) != 0: cv.imshow(line_detect.windows_name, ManyImgs(1, ([frame, binary])))
        else: cv.imshow(line_detect.windows_name, frame)
        if action == ord('q') or action == 113: break
    capture.release()
    cv.destroyAllWindows()

