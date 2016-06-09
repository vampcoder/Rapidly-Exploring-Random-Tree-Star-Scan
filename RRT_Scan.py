import cv2
img = cv2.imread('Images/obstacle.png')

cv2.imshow('image', img)
k = cv2.waitKey(0)