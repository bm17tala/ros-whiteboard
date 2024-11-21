import cv2
import numpy as np

img = cv2.imread('image.png')

black = [0,0,0]

Y, X = np.where(np.all(img==black,axis=2))

zipped = np.column_stack((X,Y))

print(zipped)