#!/usr/bin/env python

from tkinter import *
from tkinter import colorchooser, ttk
from PIL import Image, ImageDraw
import os
import math
import main

lines = []
lines.append( [] )
canvas_width = 400
canvas_height = 300

class PaintGUI:
    
    current_line = 0
    def __init__(self, master):
        self.master = master
        self.color_fg = 'Black'
        self.color_bg = 'white'
        self.old_x = None
        self.old_y = None
        self.pen_width = 5
        self.drawWidgets()
        self.c.bind('<B1-Motion>', self.paint)
        self.c.bind('<ButtonRelease-1>', self.endLine)

    def paint(self, e):
        global width
        global height

        if self.old_x and self.old_y:
            self.c.create_line(self.old_x, self.old_y, e.x, e.y, width = self.pen_width, fill = self.color_fg, capstyle='round', smooth = True)
        self.old_x = e.x
        self.old_y = e.y
        #xCoords.append(self.c.winfo_pointerx())
        #yCoords.append(self.c.winfo_pointery())
        #xCoords.append(self.c.winfo_pointerx()-285)
        #yCoords.append(self.c.winfo_pointery()-100)
        lines[self.current_line].append( (self.c.winfo_pointerx() - self.c.winfo_rootx(), 
                                        self.c.winfo_pointery() - self.c.winfo_rooty()) )
        #print("x: ", xCoords[len(xCoords)-1], " y: ", yCoords[len(yCoords)-1])


        width = self.c.winfo_width()
        height = self.c.winfo_height()

    def endLine(self, e):
        self.old_x = None
        self.old_y = None

        self.current_line += 1
        lines.append( [] )
    
    def changedW(self, width):
        self.pen_width = width
    
    def clearcanvas(self):
        self.c.delete(ALL)
    
    def brush(self):
        self.color_fg = 'Black'
        self.label.config(text="Brush Active")
    
    def eraser(self):
        self.color_fg = 'White'
        self.label.config(text="Eraser Active")

    def save(self):
        self.c.postscript(file="image.eps")
        img = Image.open('image.eps')
        img.save('image.png', 'png')
        #new_img = img.resize((200,100))
        #new_img.save('image.png', 'png')


    def close_arm(self):
        pass

    def open_arm(self):
        pass

    def send_to_ROS(self):


        canvas_xOrigin = int(canvas_width / 2)
        canvas_yOrigin = int(canvas_width / 2)

        directed_graph_img = Image.new("RGB", (canvas_width, canvas_height), color=(255, 255, 255))
        pb = ImageDraw.Draw(directed_graph_img)

        processed_lines = []

        for i in range(0, len(lines)):
            processed_lines.append([])
            for j in range(0, len(lines[i]), 10):
                processed_lines[i].append( lines[i][j] )


        for i in range(len(processed_lines)):
            for j in range(len(processed_lines[i])):
                
                fillColor = "red"
                if j == 0 or j == len(processed_lines[i]) - 1:
                    fillColor = "green"

                point_to_draw = processed_lines[i][j] + processed_lines[i][j]
                point_to_draw = list(point_to_draw)
                point_to_draw[0] -= 5
                point_to_draw[1] -= 5
                point_to_draw[2] += 5
                point_to_draw[3] += 5

                pb.ellipse( point_to_draw, fill=fillColor)
                if j > 0:
                    line_to_draw = processed_lines[i][j-1] + processed_lines[i][j]
                    print(math.degrees(math.atan2((line_to_draw[3] - line_to_draw[1]), 
                                                   line_to_draw[2] - line_to_draw[0])))
                    pb.line(line_to_draw, fill=(0, 0, 0), width=5)


                # to send these points to the robot... we need two things:
                # a. get the angle of a given line in radians. that's the direction our robot must move in.
                #    important: we'll have to be able to tell the robot to do a specific speed in x and y direction,
                #    we can use some trig to calculate these values for a given angle
                # b. get the distance the robot must travel in that angle for. we can just use distance formula
                #    of two points for that. likely use sleep() to specificy the time (distance) to draw a point?

        directed_graph_img.save("directed_graph_img.png")

        for i in range(len(processed_lines)):
            for j in range(len(processed_lines[i])):
                if j == 0:
                    main.arm_ctrl(3, 0)




    def drawWidgets(self):
        self.controls = Frame(self.master, padx=5, pady=5)
        textpw = Label(self.controls, text='Brush Width', font='Courier 12')
        textpw.grid(row=0, column=0)
        self.slider = ttk.Scale(self.controls, from_=5, to=100, command=self.changedW, orient='vertical')
        self.slider.set(self.pen_width)
        self.slider.grid(row=0, column=1)
        self.label = Label(self.controls, text='Brush Active', font='Courier 12')
        self.label.grid(row=2, column=1)
        # eraser = Button(self.controls, text ="Eraser", command=self.eraser)
        # eraser.grid(row=1,column=1)
        brush = Button(self.controls, text ="Brush", command=self.brush)
        brush.grid(row=1,column=0)
        send_to_robot = Button(self.controls, text ="Send to Robot", command=self.send_to_ROS)
        send_to_robot.grid(row=3,column=0)
        close_arm = Button(self.controls, text ="Close arm", command=self.close_arm)
        close_arm.grid(row=4,column=0)
        open_arm = Button(self.controls, text ="Open arm", command=self.open_arm)
        open_arm.grid(row=4,column=1)
        self.controls.pack(side="left")
        self.c = Canvas(self.master, width=canvas_width, height=canvas_height, bg=self.color_bg)
        self.c.pack(fill=BOTH, expand=True)

        menu = Menu(self.master)
        self.master.config(menu=menu)
        optionmenu = Menu(menu)
        menu.add_cascade(label='Menu', menu=optionmenu)
        #optionmenu.add_command(label='Brush Color', command=self.change_fg)
        optionmenu.add_command(label='Clear Canvas', command=self.clearcanvas)
        optionmenu.add_command(label='Save', command=self.save) 
        optionmenu.add_command(label='Exit', command=self.master.destroy)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_ctrl')
    #rospy.init_node('yahboom_keyboardArm')
    linear_limit = rospy.get_param('~linear_speed_limit', 1.0)
    angular_limit = rospy.get_param('~angular_speed_limit', 5.0)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #--new--
    sub_Arm = rospy.Subscriber("ArmAngleUpdate", ArmJoint, Armcallback, queue_size=1000)
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

win = Tk()
win.title("Bot Ross Drawing Utility")
PaintGUI(win)
win.mainloop()
if os.path.exists('image.eps'):
    os.remove("image.eps")
