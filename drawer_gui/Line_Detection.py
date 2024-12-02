import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from scipy.stats import linregress
import paint

xCoords = paint.xCoords
yCoords = paint.yCoords
for i in range(0, len(xCoords)//5 + 1):
    if i % 15 == 0:
        xCoords.pop(i)
        yCoords.pop(i)
        i -= 1

print(str(paint.width) + "x" + str(paint.height))
scaleX = 200 / (paint.width-4)
scaleY = 100 / (paint.height-4)

for i in range(0, len(xCoords)):
    print(xCoords[i], yCoords[i])
    xCoords[i] = 1 * int(xCoords[i] * scaleX)
    yCoords[i] = 1 * int(yCoords[i] * scaleY)
    print(xCoords[i], yCoords[i])
    print("---------")

xPoints = np.array(xCoords)
yPoints = np.array(yCoords)

graph, plot1 = plt.subplots(1)

plt.plot(xPoints, yPoints)
plt.xlim(0, 200)
plt.ylim(0, 100)
plot1.invert_yaxis()


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