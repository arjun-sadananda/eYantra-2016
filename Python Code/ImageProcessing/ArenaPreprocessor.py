"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         ArenaPreprocessor.py
* Theme:            Launch A Module
* Functions:        arena_preprocess, getTransformationMatrix, get_robot_space
* Global Variables: None
"""

import cv2
import numpy as np


"""
* Function Name:    getTransformationMatrix
* Input:            frame - (raw camera feed of the arena)
* Output:           perspective transformation matrix
* Logic:            Uses image processing techniques and finds contours for outer border to
                    get transformation matrix
                    Each process is explained in the function
* Example Call:     M = getTransformationMatrix(frame)
"""

def getTransformationMatrix(frame):
    # # flips Horizontally and Vertically: Depends on Camera Setup
    # arena = cv2.flip(frame, -1)

    # Denoising: bilateral filter Kernel size of 99 (Preferred Over medianBlur to maintain edge info)
    processed_arena = cv2.bilateralFilter(frame, 5, 99, 198)

    # To Grayscale
    processed_arena = cv2.cvtColor(processed_arena, cv2.COLOR_BGR2GRAY)

    # Increase Contrast: for better border detection
    processed_arena = cv2.equalizeHist(processed_arena)

    # Adaptive Threshold to get black thick boundary: (Used over Threshold: for lighting consideration1)
    processed_arena = cv2.adaptiveThreshold(processed_arena, 255,
                                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
                                            31, 5)

    # Morphological Operations: to remove noise
    kernel = np.ones((7, 7), np.uint8)
    processed_arena = cv2.erode(processed_arena, kernel)

    kernel = np.ones((5, 5), np.uint8)
    processed_arena = cv2.dilate(processed_arena, kernel)

    # Contour Detection
    (contours, heirarchy) = cv2.findContours(processed_arena, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Getting the contour of interest: inner edge and outer edge of the box- largest and second largest contour
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    the_outer_contour = contours[0]
    the_inner_contour = contours[1]

    # Approximating to get corners of the quadrilaterals
    peri_in = cv2.arcLength(the_inner_contour, True)
    peri_out = cv2.arcLength(the_outer_contour, True)
    in_corners = cv2.approxPolyDP(the_inner_contour, .01 * peri_in, True)
    out_corners = cv2.approxPolyDP(the_outer_contour, .01 * peri_out, True)
    if len(in_corners) != 4 and len(out_corners) != 4:
        return

    # Define result dimensions (600 X 900) therefore each block 100 X 100
    result_pts = np.float32([[0, 0], [0, 600], [900, 0], [900, 600]])

    # Sort the detected corners to align with result corners
    in_corners = in_corners[np.argsort(in_corners[:, 0, 0] + in_corners[:, 0, 1])]
    out_corners = out_corners[np.argsort(out_corners[:, 0, 0] + out_corners[:, 0, 1])]

    # corner blocks are less than 8 inches: block + center of border = 8in
    corners = (in_corners + out_corners) / 2
    source_pts = np.float32(corners)

    # cv2.drawContours(frame, [corners], -1, (255, 0, 0), 2)
    # cv2.imshow('Display'. frame)
    # cv2.waitKey(0)
    # For Debugging: cv2.drawContours(arena, corners, -1, (0, 0, 255), 5)

    # Get transformation matrix
    M = cv2.getPerspectiveTransform(source_pts, result_pts)

    return M


"""
* Function Name:    arena_preprocess
* Input:            image - (raw camera feed of the arena)
* Output:           processed_arena, warped_arena
* Logic:            Multiple openCV tricks are used to make the raw camera feed
                    as close to ideal image as possible
                    Each process is explained in the function
* Example Call:     arena_preprocess(frame, M)
"""

def arena_preprocess(frame, M):
    # Remapping to final desired result image
    processed_arena = cv2.warpPerspective(frame, M, (900, 600))

    # Make the excess black border White: ~10px thick
    in_corners = np.array([[10, 18], [10, 590], [890, 590], [890, 15]])
    h, w = processed_arena.shape[:2]
    result_mask = np.zeros((h, w), np.uint8)
    mask = np.zeros((h + 2, w + 2), np.uint8)
    cv2.drawContours(mask, [in_corners], -1, 255, 1)
    cv2.floodFill(result_mask, mask, (0, 0), 255)
    processed_arena = cv2.add(processed_arena, cv2.cvtColor(result_mask, cv2.COLOR_GRAY2BGR))

    # cv2.imshow('Display', processed_arena)
    # cv2.waitKey(0)
    warped_arena = processed_arena.copy();
    # Warped_arena: to be used for robot tracking
    # Denoising: bilateral filter
    processed_arena = cv2.bilateralFilter(processed_arena, 5, 99, 198)

    # To Make Background White:
    #   1) Invert
    arena_inv = cv2.bitwise_not(processed_arena)
    #   2) Subtract
    processed_arena = cv2.subtract(arena_inv, processed_arena)
    #   3) Invert
    processed_arena = cv2.bitwise_not(processed_arena)

    # # Color Enhancement: Does Not Help in color detection
    # ycrcb = cv2.cvtColor(processed_arena, cv2.COLOR_BGR2YCR_CB)
    # y, cr, cb = cv2.split(ycrcb)
    # cv2.equalizeHist(y, y)
    # ycrcb = cv2.merge((y, cr, cb))
    # processed_arena = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    #
    # # Shadow Removal- Not Used since Removes Shape Detail
    # shadow = cv2.cvtColor(processed_arena, cv2.COLOR_BGR2GRAY)
    # ret, shadow = cv2.threshold(shadow, 10, 255, cv2.THRESH_BINARY_INV)
    # shadow = cv2.cvtColor(shadow, cv2.COLOR_GRAY2BGR)
    # processed_arena = cv2.add(processed_arena, shadow)

    # cv2.imshow('Display', processed_arena)
    # cv2.waitKey(0)

    # Show Grid Lines
    for y in range(0, 6):
        for x in range(0, 9):
            cv2.line(processed_arena, (x * 100, y * 100), (x * 100, (y + 1) * 100), (0, 0, 0), 1)
        cv2.line(processed_arena, (0, y * 100), (900, y * 100), (0, 0, 0), 1)
    # cv2.imshow('Display', processed_arena)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # processed_arena: to be used for Object Detection
    return processed_arena, warped_arena


"""
* Function Name:    get_robot_space
* Input:            frame - (raw camera feed of the arena)
* Output:           warped portion of arena
* Logic:            Warps a portion of the arena to which the robot position
                    is mapped to avoid parallax
* Example Call:     robot_space = get_robot_space(frame)
"""


def get_robot_space(frame):
    # Denoising: bilateral filter Kernel size of 99 (Preferred Over medianBlur to maintain edge info)
    frame = cv2.bilateralFilter(frame, 5, 99, 198)

    # Define result dimensions (600 X 900) therefore each block 100 X 100
    source_pts = np.float32([[24, 56], [27, 444], [608, 47], [615, 437]]) #(576, 65) # 53,71 (53, 400) (586, 390)

    # Define result dimensions (600 X 900) therefore each block 100 X 100
    result_pts = np.float32([[0, 0], [0, 600], [900, 0], [900, 600]])

    # Get transformation matrix
    M = cv2.getPerspectiveTransform(source_pts, result_pts)

    # Remapping to final desired result image
    warped_arena = cv2.warpPerspective(frame, M, (900, 600))

    # Show Grid Lines
    for y in range(0, 6):
        for x in range(0, 9):
            cv2.line(warped_arena, (x * 100, y * 100), (x * 100, (y + 1) * 100), (0, 0, 0), 1)
        cv2.line(warped_arena, (0, y * 100), (900, y * 100), (0, 0, 0), 1)

    return warped_arena
