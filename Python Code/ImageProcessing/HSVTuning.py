# We are using this code only to get the HSV threshold values manually using trackbar

import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
cap = cv2.VideoCapture(1)
cv2.namedWindow('image', flags=1)

lower = np.array([90, 85, 110])
upper = np.array([114, 195, 255])

# create trackbars for color change
cv2.createTrackbar('Mn1', 'image', lower[0], 179, nothing)
cv2.createTrackbar('Mn2', 'image', lower[1], 255, nothing)
cv2.createTrackbar('Mn3', 'image', lower[2], 255, nothing)
cv2.createTrackbar('Mx1', 'image', upper[0], 179, nothing)
cv2.createTrackbar('Mx2', 'image', upper[1], 255, nothing)
cv2.createTrackbar('Mx3', 'image', upper[2], 255, nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

while(1):
    ret, img = cap.read()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    if k == ord('a'):
        print Mn1, Mn2, Mn3, Mx1, Mx2, Mx3

    # get current positions of four trackbars
    Mn1 = cv2.getTrackbarPos('Mn1','image')
    Mn2 = cv2.getTrackbarPos('Mn2','image')
    Mn3 = cv2.getTrackbarPos('Mn3','image')
    Mx1 = cv2.getTrackbarPos('Mx1','image')
    Mx2 = cv2.getTrackbarPos('Mx2','image')
    Mx3 = cv2.getTrackbarPos('Mx3','image')
    s = cv2.getTrackbarPos(switch,'image')

    if s == 0:
        img[:] = img
    else:
        lower = np.array([Mn1, Mn2, Mn3])
        upper = np.array([Mx1, Mx2, Mx3])
        mask = cv2.inRange(hsv, lower, upper)
        img = cv2.bitwise_and(img,img,mask=mask)
    cv2.imshow('images', img)

cv2.destroyAllWindows()
