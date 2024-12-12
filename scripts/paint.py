#!/usr/bin/env python

from tkinter import *
from tkinter import colorchooser, ttk
from PIL import Image, ImageDraw
import os
import math
import main
import rospy
from time import sleep

lines = []
lines.append( [] )
canvas_width = 400
canvas_height = 300

class PaintGUI:
    
    current_line = 0
    def __init__(self, master):
        self.master = master
        self.old_x = None
        self.old_y = None
        self.pen_width = 5
        self.drawWidgets()
        self.c.bind('<B1-Motion>', self.paint)
        self.c.bind('<ButtonRelease-1>', self.endLine)

    #Paints line on canvas and sends coordinates to lines array
    def paint(self, e):
        global width
        global height

        if self.old_x and self.old_y:
            self.c.create_line(self.old_x, self.old_y, e.x, e.y, width = self.pen_width, fill = "Black", capstyle='round', smooth = True)
        self.old_x = e.x
        self.old_y = e.y
<<<<<<< HEAD
        
        lines[self.current_line].append( (self.c.winfo_pointerx() - self.c.winfo_rootx(), 
                                        self.c.winfo_pointery() - self.c.winfo_rooty()) )
=======
        #xCoords.append(self.c.winfo_pointerx())
        #yCoords.append(self.c.winfo_pointery())
        #xCoords.append(self.c.winfo_pointerx()-285)
        #yCoords.append(self.c.winfo_pointery()-100)
        canvas_xOrigin = int(canvas_width / 2)
        canvas_yOrigin = int(canvas_height / 2)
        lines[self.current_line].append( (self.c.winfo_pointerx() - self.c.winfo_rootx(), 
                                        self.c.winfo_pointery() - self.c.winfo_rooty()))
        #print("x: ", xCoords[len(xCoords)-1], " y: ", yCoords[len(yCoords)-1])
>>>>>>> 5841f0cc2d81663f52b54348d16dae4c5e83961d


        width = self.c.winfo_width()
        height = self.c.winfo_height()

    def endLine(self, e):
        self.old_x = None
        self.old_y = None

        self.current_line += 1
        lines.append( [] )
    
<<<<<<< HEAD
    #Clears the canvas 
=======
    #def changedW(self, width):
    #    self.pen_width = width
    
>>>>>>> 5841f0cc2d81663f52b54348d16dae4c5e83961d
    def clearcanvas(self):
        # Clear on screen canvas and the underlying data structure
        # about the drawing coordinates.
        self.c.delete(ALL)
        self.current_line = 0
        lines.clear()
        lines.append( [] )
<<<<<<< HEAD
    
    #Saves canvas as png
=======

    
    #def brush(self):
    #    self.color_fg = 'Black'
    #    self.label.config(text="Brush Active")
    
    #def eraser(self):
    #    self.color_fg = 'White'
    #    self.label.config(text="Eraser Active")

>>>>>>> 5841f0cc2d81663f52b54348d16dae4c5e83961d
    def save(self):
        self.c.postscript(file="image.eps")
        img = Image.open('image.eps')
        img.save('image.png', 'png')

    #Closes arm
    def close_arm(self):
        main.arm_ctrl(4, 0)

    #Opens arm
    def open_arm(self):
        main.arm_ctrl(5, 0)

    #Prepares arm for marker mounting
    def prepare_arm(self):
        main.arm_ctrl(1, 0)

    #Processes lines and sends plans to robot for drawing
    def send_to_ROS(self):
        canvas_xOrigin = int(canvas_width / 2)
        canvas_yOrigin = int(canvas_height / 2)

        directed_graph_img = Image.new("RGB", (canvas_width, canvas_height), color=(255, 255, 255))
        pb = ImageDraw.Draw(directed_graph_img)

        processed_lines = []

        # Create more simplistic line segments to make it easier for the robot to draw
        # AKA create our graph over the drawing
        for i in range(0, len(lines)):
            processed_lines.append([])
            for j in range(0, len(lines[i]), 4):
                processed_lines[i].append( lines[i][j] )


        # Draw this graph onto an image and save it
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

        directed_graph_img.save("directed_graph_img.png")

        # to send these points to the robot... we need two things:
        # a. get the angle of a given line in radians. that's the direction our robot must move in.
        #    important: we'll have to be able to tell the robot to do a specific speed in x and y direction,
        #    we can use some trig to calculate these values for a given angle
        # b. get the distance the robot must travel in that angle for. we can just use distance formula
        #    of two points for that. likely use sleep() to specificy the time (distance) to draw a point?

        move_cmd = main.Twist()
        # pen up
        main.arm_ctrl(2, 0)
        for i in range(0, len(processed_lines)):
            startingPoint = 1

            # calculate x/y pixel distances for first segment when starting a new line
            
            if i < 1:
                # if drawing the first line in the drawing
                dx = processed_lines[i][0][0] - canvas_xOrigin
                dy = processed_lines[i][0][1] - canvas_yOrigin
            else:
                # if drawing any other line
                dx = processed_lines[i][0][0] - processed_lines[i-1][len(processed_lines[i-1])-1][0]
                dy = processed_lines[i][0][1] - processed_lines[i-1][len(processed_lines[i-1])-1][1]

            #calculate distance of line
            distance = math.sqrt( (dx ** 2) + (dy ** 2) )

            # angle of line in radians
            angle_rad = math.atan2(dy, dx)

            # X/Y Magnitudes
            move_cmd.linear.x = 0.1 * math.cos(angle_rad)  # X Magnitude
            move_cmd.linear.y = 0.1 * math.sin(angle_rad)  # Y Magnitude
            move_cmd.angular.z = 0
            print("x: ", move_cmd.linear.x, " y: ", move_cmd.linear.y, " dx: ", dx, " dy: ", dy, " angle_rad: ", angle_rad, " distance: ", distance) 

            main.pub.publish(move_cmd)

            # move for a certain amount of time according to the distance of this line
            rospy.sleep(distance / 100)

            # Publish the zero velocity command
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0
            rate = rospy.Rate(100)  # 100 Hz
            for _ in range(10):  # Publish for 1 second
               main.pub.publish(move_cmd)
               rate.sleep()

            # pen down
            main.arm_ctrl(3,0)

            for j in range(startingPoint, len(processed_lines[i])):

                #calculate x/y pixel distances for every other segment in this line
                
                # calculate x/y distance between current and last point
                dx = processed_lines[i][j][0] - processed_lines[i][j-1][0]
                dy = processed_lines[i][j][1] - processed_lines[i][j-1][1]

                #calculate distance of line
                distance = math.sqrt( (dx ** 2) + (dy ** 2) )

                #angle of line in radians
                angle_rad = math.atan2(dy, dx)

                # X/Y Magnitudes
                move_cmd.linear.x = 0.05 * math.cos(angle_rad)  # X Magnitude
                move_cmd.linear.y = 0.1 * math.sin(angle_rad)  # Y Magnitude
                move_cmd.angular.z = 0

                print("x: ", move_cmd.linear.x, " y: ", move_cmd.linear.y, " dx: ", dx, " dy: ", dy, " angle_rad: ", angle_rad, " distance: ", distance) 

                main.pub.publish(move_cmd)

                # move for a certain amount of time according to the distance of this line
                rospy.sleep(distance / 75)

            # Publish the zero velocity command
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0
<<<<<<< HEAD
=======

>>>>>>> 5841f0cc2d81663f52b54348d16dae4c5e83961d
            # Publish the zero velocity command
            rate = rospy.Rate(100)  # 100 Hz
            sleep(0.5)
            for _ in range(10):  # Publish for 1 second
               main.pub.publish(move_cmd)
               rate.sleep()

            # pen up
            main.arm_ctrl(2, 0)


    def drawWidgets(self):
        # Draw content onto the GUI
        self.controls = Frame(self.master, padx=5, pady=20, bg="#E2B1B1")
        prepare = Button(self.controls, text ="Prepare Arm", command=self.prepare_arm, width=20, height=5, bg="#73BFB8")
        prepare.grid(row=3,column=0)
        send_to_robot = Button(self.controls, text ="Send to Robot", command=self.send_to_ROS, width=20, height=5, bg="#2364AA")
        send_to_robot.grid(row=3,column=1)
        close_arm = Button(self.controls, text ="Close arm", command=self.close_arm, width=20, height=5, bg="#EA7317")
        close_arm.grid(row=4,column=0)
        open_arm = Button(self.controls, text ="Open arm", command=self.open_arm, width=20, height=5, bg="#3DA5D9")
        open_arm.grid(row=4,column=1)
        clear_canvas = Button(self.controls, text ="Clear Canvas", command=self.clearcanvas, width=20, height=5, bg= "#EC0B43")
        clear_canvas.grid(row=5,column=0)
        self.controls.pack(side="left")
        self.c = Canvas(self.master, width=canvas_width, height=canvas_height, bg="White", borderwidth="3", relief="solid")
        self.c.pack(fill=BOTH, expand=True)

# Open the Window
win = Tk()
win.title("Bot Ross Drawing Utility")
PaintGUI(win)
win.mainloop()
if os.path.exists('image.eps'):
    os.remove("image.eps")
