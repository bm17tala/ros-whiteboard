"""

Author: Max Vanderbeek, Brenden Talasco

NOTES: Whiteboard is a 4:3 ratio -> 12 inches wide, 9 inches tall

"""

from tkinter import *
from tkinter import colorchooser, ttk
from PIL import Image, ImageDraw
import os
import math

lines = []
lines.append( [] )
canvas_width = 400
canvas_height = 300

class main:
    
    current_line = 0
    def __init__(self, master):
        self.master = master
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
            self.c.create_line(self.old_x, self.old_y, e.x, e.y, width = self.pen_width, fill = "Black", capstyle='round', smooth = True)
        self.old_x = e.x
        self.old_y = e.y
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

    
    def clearcanvas(self):
        self.c.delete(ALL)
        self.current_line = 0
        lines.clear()
        lines.append( [] )

    def close_arm(self):
        pass

    def open_arm(self):
        pass

    def prepare_arm(self):
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


    def drawWidgets(self):
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
        

win = Tk()
win.title("Paint App")
main(win)
win.mainloop()
if os.path.exists('image.eps'):
    os.remove("image.eps")