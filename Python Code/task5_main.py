"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         task5_main.py
* Theme:            Launch A Module
* Functions:        main
* Global Variables: None
"""

import cv2

import Communication.RobotCommunication as RC
import ImageProcessing.ArenaPreprocessor as AP
import ImageProcessing.ObjectDetector as OD
import ImageProcessing.RobotTracking as RT
from MotionExecution import MotionPlanner as MP

"""
* Function Name:    main
* Input:            image - (raw camera feed of the arena)
* Output:           state - (Success or Fail)
* Logic:            Picks one object and drops it at the goal
                    Tasks:
                        1) Preprocessing
                        2) Object Detection
                        3) Robot Tracking
                        3) Data Handling (Grid)
                        4) Match Making
                        5) Motion Planning- (Explained in MotionPlanning.py)
* Example Call:     main(frame)
"""


def main(img):

    arena_transformation_matrix = AP.getTransformationMatrix(img)
    if arena_transformation_matrix is None:
        return
    cv2.namedWindow('Matches Made', cv2.WINDOW_NORMAL)

    matches_history = [0, 0, 0, 0, 0, 0, 0]

    for x in range(0, 30):
        ret, frame = videoCapture.read()
        if not ret:
            continue
        frame = cv2.flip(frame, -1)

        # preprocessed_arena: used for object detection
        # warped_arena: used for robot tracking
        processed_arena, warped_arena = AP.arena_preprocess(frame, arena_transformation_matrix)
        # robot_node: must be ignored while object tracking
        robot_space = AP.get_robot_space(frame)
        robot_node, (x, y), robot_head, heading_angle = RT.robot_track(robot_space)
        # print (x, y)

        # grid: to hold information from object_detector
        grid = OD.object_detector(processed_arena, robot_node)

        # matches_made: number of matches found
        matches_made = grid.match_goal_to_object
        matches_history[matches_made] += 1

        # display_info
    probable_no_of_matches = 0
    # for x in range(0, 6):
    #     if matches_history[x] > matches_history[probable_no_of_matches]:
    #         probable_no_of_matches = x
    for x in range(0, 6):
        if matches_history[x] != 0:
            probable_no_of_matches = x
    # print probable_no_of_matches
    # print 'Probably'
    # print matches_history

    while 1:
        ret, frame = videoCapture.read()
        if not ret:
            continue
        frame = cv2.flip(frame, -1)

        # preprocessed_arena: used for object detection
        # warped_arena: used for robot tracking
        processed_arena, warped_arena = AP.arena_preprocess(frame, arena_transformation_matrix)
        # robot_node: must be ignored while object tracking
        robot_space = AP.get_robot_space(frame)
        robot_node, (x, y), robot_head, heading_angle = RT.robot_track(robot_space)
        # print (x, y)

        # grid: to hold information from object_detector
        grid = OD.object_detector(processed_arena, robot_node)

        # matches_made: number of matches found
        matches_made = grid.match_goal_to_object

        # display_info

        display = RT.show_robot(processed_arena, (x, y), heading_angle)
        display = grid.show_objects(display)
        display = grid.show_matches(display)

        cv2.imshow('Matches Made', display)

        if matches_made == probable_no_of_matches:#len(grid.goals): #2: #
            RC.start()
            RC.send('s')    # Start
            # RC.send('g+000+000')
            cv2.waitKey(500)
            while matches_made > 0:
                ret, frame = videoCapture.read()
                if not ret:
                    continue
                frame = cv2.flip(frame, -1)
                robot_space = AP.get_robot_space(frame)
                robot_node, _, robot_head, _ = RT.robot_track(robot_space)
                MP.deliver_an_object(robot_node, robot_head, videoCapture, grid, arena_transformation_matrix)
                matches_made -= 1
                # print 'Matches = ', matches_made

            return

        c = cv2.waitKey(1) & 0xFF
        if c == ord('q'):
            return
        if c == ord('p'):
            cv2.imshow('RobotSpace', robot_space)
            cv2.waitKey(0)
            cv2.destroyWindow('RobotSpace')


'''
Below part of program will run when ever this file (task1_main.py) is run directly from terminal/Idle prompt.

'''
if __name__ == '__main__':
    videoCapture = cv2.VideoCapture(1)
    cv2.namedWindow('Display', cv2.WINDOW_AUTOSIZE)

    # For Testing Communication
    # RC.start()
    # RC.send('s')
    # RC.send('g+000+000')
    # cv2.waitKey(1000)
    # RC.send('g+200+200')
    # cv2.waitKey(2000)
    # RC.send('g+000+000')
    # cv2.waitKey(2000)
    # RC.send('p')
    # cv2.waitKey(3000)
    # RC.send('g+000+000')
    # cv2.waitKey(2000)
    #
    # RC.send('d')
    # cv2.waitKey(3000)
    # RC.send('e')
    # cv2.waitKey(2000)
    # cv2.waitKey(0)
    i = 0

    while 1:
        ret, image = videoCapture.read()
        if not ret:
            continue

        # flips Horizontally and Vertically: Depends on Camera Setup
        image = cv2.flip(image, -1)

        cv2.imshow('Display', image)

        c = cv2.waitKey(10) & 0xFF
        i += 1
        if i == 50:
            main(image)
            # ropbot_node, (x, y), robot_head, heading_angle = RT.robot_track(image)
            # print (x, y)
            break

        if c == ord('q'):
            break

    videoCapture.release()
    cv2.destroyAllWindows()
