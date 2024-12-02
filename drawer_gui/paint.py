from tkinter import *
from tkinter import colorchooser, ttk
from PIL import Image
import os

class main:
    global xCoords
    global yCoords
    xCoords = []
    yCoords = []
    def __init__(self, master):
        self.master = master
        self.color_fg = 'Black'
        self.color_bg = 'white'
        self.old_x = None
        self.old_y = None
        self.pen_width = 5
        self.drawWidgets()
        self.c.bind('<B1-Motion>', self.paint)
        self.c.bind('<ButtonRelease-1>', self.reset)

    def paint(self, e):
        if self.old_x and self.old_y:
            self.c.create_line(self.old_x, self.old_y, e.x, e.y, width = self.pen_width, fill = self.color_fg, capstyle='round', smooth = True)
        self.old_x = e.x
        self.old_y = e.y
        xCoords.append(self.c.winfo_pointerx()-285)
        yCoords.append(self.c.winfo_pointery()-100)
        global width
        global height
        width = self.c.winfo_width()
        height = self.c.winfo_height()

    def reset(self, e):
        self.old_x = None
        self.old_y = None
    
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
        new_img = img.resize((200,100))
        new_img.save('image.png', 'png')

    def drawWidgets(self):
        self.controls = Frame(self.master, padx=5, pady=5)
        textpw = Label(self.controls, text='Brush Width', font='Courier 12')
        textpw.grid(row=0, column=0)
        self.slider = ttk.Scale(self.controls, from_=5, to=100, command=self.changedW, orient='vertical')
        self.slider.set(self.pen_width)
        self.slider.grid(row=0, column=1)
        self.label = Label(self.controls, text='Brush Active', font='Courier 12')
        self.label.grid(row=2, column=1)
        eraser = Button(self.controls, text ="Eraser", command=self.eraser)
        eraser.grid(row=1,column=1)
        brush = Button(self.controls, text ="Brush", command=self.brush)
        brush.grid(row=1,column=0)
        self.controls.pack(side="left")
        self.c = Canvas(self.master, width=500, height=400, bg=self.color_bg)
        self.c.pack(fill=BOTH, expand=True)

        menu = Menu(self.master)
        self.master.config(menu=menu)
        optionmenu = Menu(menu)
        menu.add_cascade(label='Menu', menu=optionmenu)
        #optionmenu.add_command(label='Brush Color', command=self.change_fg)
        optionmenu.add_command(label='Clear Canvas', command=self.clearcanvas)
        optionmenu.add_command(label='Save', command=self.save) 
        optionmenu.add_command(label='Exit', command=self.master.destroy)
        



win = Tk()
win.title("Paint App")
main(win)
win.mainloop()
if os.path.exists('image.eps'):
    os.remove("image.eps")