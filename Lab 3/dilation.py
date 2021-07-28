import cv2
import numpy as np 
import matplotlib.pyplot as plt

img = cv2.imread('dilation.png', 0)
# rows,col,ch = img.shape
# img = img[228:248, 1:441, 1:3]
# rows,col,ch = img.shape
# kernel = np.ones((5,5),np.uint8)
# edge = cv2.Canny(img, 300, 445)
# dilation = cv2.dilate(edge,kernel, iterations =5)
centred_img = cv2.copyMakeBorder(img, 0,0,0,10,cv2.BORDER_CONSTANT,value =1)

plt.subplot(211),cv2.imshow('image',centred_img)
plt.subplot(212), cv2.imshow('image', img)

cv2.waitKey(0)
