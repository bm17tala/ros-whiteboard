"""

Authors: Brenden Talasco, Max Vanderbeek

NOTES: Whiteboard is a 4:3 ratio -> 12 inches wide, 9 inches tall

"""

#import cv2
import numpy as np
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
#from scipy.stats import linregress
import paint

canvas_width = 500
canvas_height = 400

canvas_xOrigin = int(canvas_width / 2)
canvas_yOrigin = int(canvas_width / 2)

#xCoords = paint.xCoords
#yCoords = paint.yCoords
lines = paint.lines


directed_graph_img = Image.new("RGB", (canvas_width, canvas_height), color=(255, 255, 255))
pb = ImageDraw.Draw(directed_graph_img)

#for i in range(0, len(xCoords)//5 + 1):
#   if i % 2 == 0:
#       xCoords.pop(i)
#       yCoords.pop(i)
#       i -= 1

processed_lines = []
#current_processed_line = 0

for i in range(0, len(lines)):
    processed_lines.append([])
    for j in range(0, len(lines[i]), 10):
        #print(i)
        processed_lines[i].append( lines[i][j] )
        #print("TEST x: ", processed_xCoords[len(processed_xCoords)-1], " y: ", processed_yCoords[len(processed_yCoords)-1])



#print(str(paint.width) + "x" + str(paint.height))
#scaleX = 200 / (paint.width-4)
#scaleY = 100 / (paint.height-4)

for i in range(len(processed_lines)):
    for j in range(len(processed_lines[i])):
        #print(processed_xCoords[i], processed_yCoords[i])
        
        fillColor = "red"
        if j == 0 or j == len(processed_lines[i]) - 1:
            fillColor = "green"

        point_to_draw = processed_lines[i][j] + processed_lines[i][j]
        point_to_draw = list(point_to_draw)
        point_to_draw[0] -= 5
        point_to_draw[1] -= 5
        point_to_draw[2] += 5
        point_to_draw[3] += 5


        


        #pb.ellipse((processed_xCoords[i]-5, processed_yCoords[i]-5, processed_xCoords[i]+5, processed_yCoords[i]+5), fill=fillColor)
        #if i > 0:
        #    pb.line((processed_xCoords[i-1], processed_yCoords[i-1], processed_xCoords[i], processed_yCoords[i]), fill=(0, 0, 0), width=5)
        pb.ellipse( point_to_draw, fill=fillColor)
        if j > 0:
            line_to_draw = processed_lines[i][j-1] + processed_lines[i][j]
            print(line_to_draw)
            pb.line(line_to_draw, fill=(0, 0, 0), width=5)

        #xCoords[i] = 1 * int(xCoords[i] * scaleX)
        #yCoords[i] = 1 * int(yCoords[i] * scaleY)
        #print(xCoords[i], yCoords[i])

        #print("---------")

#xPoints = np.array(xCoords)
#yPoints = np.array(yCoords)

#graph, plot1 = plt.subplots(1)

#plt.plot(xPoints, yPoints)
#plt.xlim(0, 200)
#plt.ylim(0, 100)
#plot1.invert_yaxis()

directed_graph_img.save("directed_graph_img.png")


#plt.show()


""""
point = []
point.append(None)
point.append(None)
line = []
lines = []

for i in range(0, len(xCoords)-1):
    if abs(xCoords[i] - xCoords[i+1]) > 10 or abs(yCoords[i] - yCoords[i+1]) > 10:
        point[0] = xCoords[i]
        point[1] = yCoords[i]
        line.append(point)
    else:
        lines.append(line)
        line = []
    
print(lines)
"""