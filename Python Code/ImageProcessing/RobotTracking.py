"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         RobotTracking.py
* Theme:            Launch A Module
* Functions:        robot_track, show_robot
* Global Variables: None
"""

import cv2
import numpy as np
import math

"""
* Function Name:    robot_track
* Input:            image - (raw camera feed of the arena)
* Output:           (X, Y), (x, y), heading, heading_angle: real values and node(int)
* Logic:            Track Robot using HSV inRange
                    and derive heading and position (pose of the robot)
* Example Call:     robot_track(warped_arena)
"""


def robot_track(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # found in advance using HSVTuning.py
    lower_cyan = np.array([90, 85, 110])
    upper_cyan = np.array([114, 195, 255])

    lower_yellow = np.array([8, 80, 150])
    upper_yellow = np.array([50, 222, 235])  # in Sunligh: 11 66 216 36 164 249

    mask_cyan = cv2.inRange(hsv, lower_cyan, upper_cyan)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Morphological Operations: to remove noise
    kernel = np.ones((3, 3), np.uint8)
    mask_cyan = cv2.dilate(mask_cyan, kernel)
    mask_yellow = cv2.dilate(mask_yellow, kernel)

    # kernel = np.ones((7, 7), np.uint8)
    # processed_arena = cv2.erode(processed_arena, kernel)

    # Debugging:
    # cv2.imshow('Display', mask_cyan)
    # cv2.waitKey(0)
    #
    # cv2.imshow('Display', mask_yellow)
    # cv2.waitKey(0)

    # Find all outer contours
    cnts_cy = cv2.findContours(mask_cyan.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_cy = cnts_cy[0]

    cnts_ye = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_ye = cnts_ye[0]

    # Find largest of the contours
    cnts_cy = sorted(cnts_cy, key=cv2.contourArea, reverse=True)[:5]
    cnts_ye = sorted(cnts_ye, key=cv2.contourArea, reverse=True)[:5]

    # Using Moments find Pose of the robot
    M_cy = cv2.moments(cnts_cy[0])
    cX_cy = int(M_cy["m10"] / M_cy["m00"])
    cY_cy = int(M_cy["m01"] / M_cy["m00"])
    if len(cnts_ye) > 0:
        M_ye = cv2.moments(cnts_ye[0])
    cX_ye = int(M_ye["m10"] / M_ye["m00"])
    cY_ye = int(M_ye["m01"] / M_ye["m00"])

    # (x, y): robot position: in pixels (0, 0) to (900, 600)
    x = (cX_ye + cX_cy) / 2
    y = (cY_ye + cY_cy) / 2

    # (X, Y): robot_node (0, 0) to (9, 6)
    X = int(x / 100)
    Y = int(y / 100)

    # heading_angle: actual heading of robot in degrees 0deg to 360deg
    heading_angle = math.atan2((cY_cy - cY_ye), (cX_cy - cX_ye))

    # heading: approximated {0: 0deg, 1: 90deg, 2: 180deg, 3: 270deg}
    if 3*45/2 < math.degrees(heading_angle) < 5*45/2:
        heading = 3
    elif -45/2 < math.degrees(heading_angle) < 45/2:
        heading = 0
    elif -3*45/2 > math.degrees(heading_angle) > -5*45/2:
        heading = 1
    elif -45/2 > math.degrees(heading_angle) > -3*45/2:
        heading = 0.5
    elif -5 * 45 / 2 > math.degrees(heading_angle) > -7 * 45 / 2:
        heading = 1.5
    elif 45/2 < math.degrees(heading_angle) < 3*45/2:
        heading = 3.5
    elif 5 * 45/2 < math.degrees(heading_angle) < 7*45/2:
        heading = 2.5
    else:
        heading = 2

    return (X, Y), (x, y), heading, heading_angle


"""
* Function Name:    show_robot
* Input:            arena_image , position, heading
* Output:           display
* Logic:            draws relevant information on the arena_image
* Example Call:     show_robot(arena, pos, head)
"""

def show_robot(arena, pos, head):
    disp = arena.copy()
    (x, y) = pos
    cv2.circle(disp, pos, 20, (255, 0, 255), 3)
    head = math.degrees(head)
    # if head > 180:
    #     head -= 360
    cv2.ellipse(disp, pos, (20, 20), head, -30, 30, (130, 0, 75), 4)
    cv2.ellipse(disp, pos, (30, 30), head, -30, 30, (112, 25, 25), 4)
    cv2.ellipse(disp, pos, (40, 40), head, -30, 30, (128, 0, 0), 4)
    cv2.ellipse(disp, pos, (50, 50), head, -30, 30, (225, 105, 65), 3)
    cv2.ellipse(disp, pos, (60, 60), head, -30, 30, (255, 191, 0), 2)

    return disp
