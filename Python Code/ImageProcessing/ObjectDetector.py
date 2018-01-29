"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         ObjectDetector.py
* Theme:            Launch A Module
* Functions:        detect_shape, label_color, object_detector, SIZE_THRESH
* Global Variables: APPROX_ALLOWANCE, SQUARE_EDGE_ALLOWANCE, OBJECT_THRESHOLD, AREA_THRESH,
                    OBSTACLE_AREA_THRESH, PA_RATIO_THRESH
"""

import cv2
import numpy as np
from scipy.spatial import distance as dist
import Planner.Grid as Grid

APPROX_ALLOWANCE = 0.05
SQUARE_EDGE_ALLOWANCE = .8

"""
* Function Name: detect_shape
* Input: contour
* Output: shape (either Triangle, Square or Circle)
* Logic: aprroximates contour by polygon with number of points which will determine shape
* Example Call: shape = detect_shape(contour)
"""


def detect_shape(c):
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, APPROX_ALLOWANCE * peri, True)

    if len(approx) == 3:
        shape = "Triangle"
    elif len(approx) == 4:
        a = dist.euclidean(approx[0], approx[1])
        b = dist.euclidean(approx[1], approx[2])
        c = dist.euclidean(approx[2], approx[3])
        d = dist.euclidean(approx[3], approx[0])
        m = min([a, b, c, d])
        if m < SQUARE_EDGE_ALLOWANCE * (a + b + c + d) / 4:
            shape = "Triangle"
        else:
            shape = "Square"
    else:
        shape = "Circle"

    return shape


"""
* Function Name: label_colour
* Input: image, mask
* Output: color (Red, Green or Blue)
* Logic: applies mask which only includes contour
*        takes the average B, G and R values and decides color of shape
* Example Call: color = label_color(image, mask)
"""


def label_color(image, mask):
    kernel = np.ones((3, 3), np.uint8)
    block_mask = cv2.erode(mask, kernel)
    block_mask = cv2.cvtColor(block_mask, cv2.COLOR_GRAY2BGR)
    color_sample = cv2.bitwise_and(image, block_mask)
    color_sample = cv2.pyrDown(color_sample);
    b, g, r = cv2.split(color_sample)
    b_avg = b.mean()
    g_avg = g.mean()
    r_avg = r.mean()

    if b_avg > g_avg and b_avg > r_avg:
        return "Blue"  # + str(b_avg)
    elif g_avg > b_avg and g_avg > r_avg:
        return "Green"  # + str(g_avg)
    else:
        return "Red"  # + str(r_avg)


OBJECT_THRESHOLD = 240
AREA_THRESH = 80
OBSTACLE_AREA_THRESH = 1000
PA_RATIO_THRESH = .6

"""
* Function Name: SIZE_THRESH
* Input: shape
* Output: threshold
* Logic: sets the threshold based on which shape
* Example Call: thresh = SIZE_THRESH(shape)
"""


def SIZE_THRESH(shape):
    if shape == 'Triangle':
        # return 50
        return 180
    elif shape == 'Square':
        # return 100
        return 350
    else:
        return 300


"""
* Function Name: object_detector
* Input: arena, robot_node
* Output: stores list of objects
* Logic: finds contours in each block of the grid, calls shape detetor and color detector and decides size
* Example Call: object_detector(arena, robot_node)
"""


def object_detector(arena, robot_node):
    grid = Grid.GridHandler()

    for y in range(0, 6):
        for x in range(0, 9):

            if (x, y) == robot_node:
                continue
            block = arena[y * 100: (y + 1) * 100, x * 100: (x + 1) * 100]
            gray_block = cv2.cvtColor(block, cv2.COLOR_BGR2GRAY)

            ret, block_mask = cv2.threshold(gray_block, OBJECT_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

            (contours, _) = cv2.findContours(block_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) == 0:
                continue

            object_detected = False
            for c in contours:
                M = cv2.moments(c)
                area = M["m00"]
                if area < AREA_THRESH:
                    continue

                if (cv2.arcLength(c, True) / area) > .6:
                    continue

                cX = int(M["m10"] / area)
                cY = int(M["m01"] / area)

                if cX < 20 or cX > 80 or cY < 20 or cY > 80:
                    continue

                h, w = block.shape[:2]
                result_mask = np.zeros((h, w), np.uint8)
                mask = np.zeros((h + 2, w + 2), np.uint8)
                cv2.drawContours(mask, [c], -1, 255, 1)
                cv2.floodFill(result_mask, mask, (cX, cY), 255)

                color = label_color(block, result_mask)

                shape = detect_shape(c)

                size = 'Large' if area > SIZE_THRESH(shape) else 'Medium'

                if color == 'Red' and shape == 'Square' and size == 'Large' and area > OBSTACLE_AREA_THRESH:
                    cv2.putText(arena, 'Obstacle', (x * 100 + 10, y * 100 + 80), 1, 1, (0, 0, 0), 1)
                    grid.obstacles.append((x, y))
                elif x == 0:
                     grid.goals[(0, y)] = [color, shape, size]
                else:
                    grid.objects[(x, y)] = [color, shape, size]

                object_detected = True


    return grid
