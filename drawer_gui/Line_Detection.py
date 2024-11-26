import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from scipy.stats import linregress

image = Image.open("image.png")
width, height = image.size
top_left = image.crop((0, 0,width/2,height/2))
top_left.save('top_left.png', 'png')

top_right = image.crop((width/2, 0,width,height/2))
top_right.save('top_right.png', 'png')

bot_left = image.crop((0, height/2,width/2,height))
bot_left.save('bot_left.png', 'png')

bot_right = image.crop((width/2, height/2,width,height))
bot_right.save('bot_right.png', 'png')

top_left1 = cv2.imread('top_left.png')
top_right1 = cv2.imread('top_right.png')
bot_left1 = cv2.imread('bot_left.png')
bot_right1 = cv2.imread('bot_right.png')

black = [0,0,0]
coords = np.where(np.all(top_left1==black,axis=2))
coords1y = coords[0]
coords1x = coords[1]
coords = np.where(np.all(top_right1==black,axis=2))
coords2y = coords[0]
coords2x = coords[1]
coords = np.where(np.all(bot_left1==black,axis=2))
coords3y = coords[0]
coords3x = coords[1]
coords = np.where(np.all(bot_right1==black,axis=2))
coords4y = coords[0]
coords4x = coords[1]


reg = linregress(coords1x,coords1y)
equation = reg.slope*coords1x + reg.intercept
graph, plot1 = plt.subplots(1)

plt.plot(coords1x, equation,)
plt.xlim(0, top_left.width)
plt.ylim(0, top_left.height)
plt.scatter()
#plt.scatter(coords1x, coords1y)
plot1.invert_yaxis()

plt.show()
