"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         MotionPlanner.py
* Theme:            Launch A Module
* Functions:        deliver_an_object, execute_path
* Global Variables: BLOCK_LENGTH
"""

import math

import ImageProcessing.ArenaPreprocessor as AP
import Communication.RobotCommunication as RC
import ImageProcessing.RobotTracking as RT
import cv2

import MotionExecution.Controller as Controller
import Planner.PathPlanner as PathPlanner
import MotionExecution.RobotInstructions as RI

"""
* Function Name: deliver_an_object
* Input: robot_node, robot_head, videoCapture, grid, transformation_matrix
* Output: the series of motions which robot must perform
* Logic: starts the communication with the robot
*        finds the nearest goal
*        plans the path after choosing match
*        sends instructions to perform the motion
*        picks up the object after motion is complete
*        plans the next path to goal and drops object there
* Example Call: deliver_an_object(robot_node, robot_head, videoCapture, grid, M)
"""

BLOCK_LENGTH = 100


def deliver_an_object(robot_node, robot_head, videoCapture, grid, M):
    valid_goal_nodes = {}  # goal: dist
    print 'Matches', grid.matches
    for goal_node in grid.matches.keys():
        # Assign Matching Object Node
        object_node = grid.matches[goal_node]

        # PathPlan to GOTO Object
        path_to_object = PathPlanner.plan_path(robot_node, object_node, grid)
        path_to_goal = PathPlanner.plan_path(object_node, goal_node, grid)

        if path_to_object is not None and path_to_goal is not None:
            man_dist = grid.heuristic(robot_node, object_node) + grid.heuristic(object_node, goal_node)
            valid_goal_nodes[goal_node] = man_dist

    # choosing goal node with minimum distance to be travelled
    goal_node = min(valid_goal_nodes.items(), key=lambda x: x[1])[0]
    object_node = grid.matches[goal_node]
    print 'Chosen', goal_node, object_node

    path_to_object = PathPlanner.plan_path(robot_node, object_node, grid)
    # if path_to_object exists
    if path_to_object:
        # Execute GOTO object
        path_to_object = PathPlanner.plan_path(robot_node, object_node, grid)
        state, robot_head = execute_path(robot_node, robot_head, path_to_object, videoCapture, M, grid)

        if state is True:
            # Pick Command
            cv2.waitKey(500)
            RC.send('a')
            cv2.waitKey(1000)

            state = capture_goal(object_node, videoCapture, M)

            if state is True:

                # similarly execute path to goal and drop the object
                path_to_goal = PathPlanner.plan_path(object_node, goal_node, grid)

                if path_to_goal:
                    state, _ = execute_path(object_node, robot_head, path_to_goal, videoCapture, M, grid)
                    if state is True:
                        cv2.waitKey(50)
                        RC.send('d')
                        cv2.waitKey(4000)
                        del grid.matches[goal_node]
                        del grid.occupied[grid.occupied.index(object_node)]
                        if len(grid.matches) == 0:  # exit when all matches are made
                            RC.send_velocity(0, 0)
                            print 'Waiting to end'
                            RC.send('e')
                            print 'end'
                            return False
                        return True
                    else:
                        return False
                else:
                    return False
        else:
            return False


"""
* Function Name: execute_path
* Input: robot_node, robot_head, path, videoCapture, transformation_matrix, grid
* Output: robot would have reached desired position
* Logic: calls PID controller to reach goal at correct angle
         also sends pick up and drop signals to robot
* Example Call: execute_path(robot_node, robot_head, path, videoCapture, M, grid)
"""


def execute_path(robot_node, robot_head, path, videoCapture, M, grid):
    controller = Controller.Controller()
    instructions = RI.path_to_instructions(robot_node, robot_head, path)

    for i in instructions.keys():
        controller.e_k_i = 0.0
        controller.e_k_1 = 0.0
        next_node = instructions[i][1]
        state = False
        if instructions[i][0] == 'F':
            desired_head = RI.diff_to_dir(next_node, robot_node)
            while state is False:
                ret, frame = videoCapture.read()
                if not ret:
                    continue
                frame = cv2.flip(frame, -1)

                # tracking the robot
                robot_space = AP.get_robot_space(frame)
                _, warped_arena = AP.arena_preprocess(frame, M)
                _, (x, y), robot_head, heading_angle = RT.robot_track(robot_space)

                (goal_node_x, goal_node_y) = next_node
                goal_y = int(goal_node_y * BLOCK_LENGTH + 50)
                goal_x = int(goal_node_x * BLOCK_LENGTH + 50)

                # Accounting for OverShoot
                if desired_head == 0:
                    goal_x -= 30
                if desired_head == 1:
                    goal_y += 30
                if desired_head == 2:
                    goal_x += 30
                if desired_head == 3:
                    goal_y -= 30

                if desired_head in [0.5, 3.5]:
                    goal_x -= 15
                if desired_head in [0.5, 1.5]:
                    goal_y += 15
                if desired_head in [1.5, 2.5]:
                    goal_x += 15
                if desired_head in [2.5, 3.5]:
                    goal_y -= 15

                (v_l, v_r), theta_goal, state = controller.go_to_goal((x, y), heading_angle, (goal_x, goal_y))

                RC.send_velocity(v_l, v_r)

                # add important info to the display
                display = PathPlanner.show_path(warped_arena, path)
                display = RT.show_robot(display, (x, y), heading_angle)
                cv2.circle(display, (goal_x, goal_y), 20, (0, 0, 0), 3)
                cv2.putText(display, 'GOTO', (10, 30), 1, 2, (0, 0, 255), 2)
                cv2.putText(display, 'v_L:' + str(v_l) + ' v_R:' + str(v_r), (10, 60), 1, 2, (255, 0, 255), 2)
                cv2.putText(display, 'theta: ' + str(math.degrees(heading_angle)), (10, 90), 1, 2, (255, 0, 255), 2)
                cv2.putText(display, 'theta_goal: ' + str(math.degrees(theta_goal)), (10, 120), 1, 2, (255, 0, 255), 2)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    RC.send_velocity(0, 0)
                    RC.send('e')
                    return False, robot_head
                cv2.imshow('Display', display)
                # plt.plot(controller.e_k_list)         # plotting graph of error
                # plt.show()

        if instructions[i][0] == 'T':
            turn = 'A'
            rotation = None
            ret, frame = videoCapture.read()
            if i == 0:
                t = 0
                while t <= 80:
                    if t % 10 == 0:
                        ret, frame = videoCapture.read()
                        if not ret:
                            continue
                    t += 1
                    cv2.waitKey(1)

                # tracking the robot
                frame = cv2.flip(frame, -1)
                robot_space = AP.get_robot_space(frame)
                robot_node, robot_pos, _, heading = RT.robot_track(robot_space)
                desired_head = RI.diff_to_dir(next_node, robot_node)


                if desired_head is not None:
                    rotation = abs(robot_head - desired_head)
                    # print 'Rotation', rotation
                    # if 180 deg turn insist direction of turn if obstacles around
                    turn = 'A'  # Any
                    if rotation == 2:
                        (x, y) = robot_node

                        # deciding left and right odes
                        if robot_head == 0:
                            left_node = (x, y - 1)
                            right_node = (x, y + 1)
                        elif robot_head == 1:
                            left_node = (x - 1, y)
                            right_node = (x + 1, y)
                        elif robot_head == 2:
                            left_node = (x, y + 1)
                            right_node = (x, y - 1)
                        elif robot_head == 3:
                            left_node = (x + 1, y)
                            right_node = (x - 1, y)

                        # choose left or right U turn based on nearby obstacles
                        if grid.is_obstacle(left_node) and not grid.is_obstacle(right_node):
                            turn = 'R'
                        elif grid.is_obstacle(right_node) and not grid.is_obstacle(left_node):
                            turn = 'L'
                        elif grid.is_obstacle(right_node) and grid.is_obstacle(left_node):
                            # RC.send('x')
                            # turn = 'X'
                            (pos_x, pos_y) = robot_pos

                            del_x = pos_x % 100 - 50
                            del_y = pos_y % 100 - 50

                            if robot_head == 0:
                                if del_y > 0:
                                    turn = 'L'
                                else:
                                    turn = 'R'
                            elif robot_head == 1:
                                if del_x > 0:
                                    turn = 'L'
                                else:
                                    turn = 'R'
                            elif robot_head == 2:
                                if del_y < 0:
                                    turn = 'L'
                                else:
                                    turn = 'R'
                            elif robot_head == 3:
                                if del_x < 0:
                                    turn = 'L'
                                else:
                                    turn = 'R'

                        # print 'Turn', turn
            # controller.e_k_list = []
            while state is False:
                ret, frame = videoCapture.read()
                if not ret:
                    continue
                frame = cv2.flip(frame, -1)

                # tracking the robot
                robot_space = AP.get_robot_space(frame)
                _, warped_arena = AP.arena_preprocess(frame, M)
                robot_node, robot_pos, robot_head, heading_angle = RT.robot_track(robot_space)

                (goal_node_x, goal_node_y) = next_node
                goal_y = goal_node_y * BLOCK_LENGTH + 50
                goal_x = goal_node_x * BLOCK_LENGTH + 50

                # get velocities from controller
                (v_l, v_r), theta_goal, state = controller.hard_turn_to_theta(robot_pos, heading_angle,
                                                                              (goal_x, goal_y), rotation, turn)

                RC.send_velocity(v_l, v_r)

                # add important info to the display
                display = PathPlanner.show_path(warped_arena, path)
                display = RT.show_robot(display, robot_pos, heading_angle)
                cv2.line(display, robot_pos, (goal_x, goal_y), (0, 0, 0), 2)
                cv2.putText(display, 'Alligning to Next_Node', (10, 30), 1, 2, (0, 0, 255), 2)
                cv2.putText(display, 'v_L:' + str(v_l) + ' v_R:' + str(v_r), (10, 60), 1, 2, (255, 0, 255), 2)
                cv2.putText(display, 'theta: ' + str(math.degrees(heading_angle)), (10, 90), 1, 2, (255, 0, 255), 2)
                cv2.putText(display, 'theta_goal: ' + str(math.degrees(theta_goal)), (10, 120), 1, 2, (255, 0, 255), 2)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    RC.send_velocity(0, 0)
                    RC.send('e')
                    return False, robot_head
                cv2.imshow('Display', display)
                # plt.plot(controller.e_k_list)     # for plotting graph of error
                # plt.show()
        cv2.waitKey(300)

    return True, robot_head


def capture_goal(goal_node, videoCapture, M):
    controller = Controller.Controller()
    state = False

    (goal_node_x, goal_node_y) = goal_node
    goal_y = int(goal_node_y * BLOCK_LENGTH + 50)
    goal_x = int(goal_node_x * BLOCK_LENGTH + 50)

    while state is False:
        ret, frame = videoCapture.read()
        if not ret:
            continue
        frame = cv2.flip(frame, -1)

        # tracking the robot
        robot_space = AP.get_robot_space(frame)
        _, warped_arena = AP.arena_preprocess(frame, M)
        _, (x, y), robot_head, heading_angle = RT.robot_track(robot_space)

        temp_goal_x, temp_goal_y = goal_x, goal_y

        # Accounting for OverShoot
        if robot_head == 0:
            temp_goal_x = goal_x - 40
        if robot_head == 1:
            temp_goal_y = goal_y + 40
        if robot_head == 2:
            temp_goal_x = goal_x + 40
        if robot_head == 3:
            temp_goal_y = goal_y - 40

        (v_l, v_r), theta_goal, state = controller.capture_object((x, y), heading_angle, (temp_goal_x, temp_goal_y))

        RC.send_velocity(v_l, v_r)

        # add important info to the display
        display = RT.show_robot(warped_arena, (x, y), heading_angle)
        cv2.circle(display, (goal_x, goal_y), 20, (0, 0, 0), 3)
        cv2.putText(display, 'GOTO', (10, 30), 1, 2, (0, 0, 255), 2)
        cv2.putText(display, 'v_L:' + str(v_l) + ' v_R:' + str(v_r), (10, 60), 1, 2, (255, 0, 255), 2)
        cv2.putText(display, 'theta: ' + str(math.degrees(heading_angle)), (10, 90), 1, 2, (255, 0, 255), 2)
        cv2.putText(display, 'theta_goal: ' + str(math.degrees(theta_goal)), (10, 120), 1, 2, (255, 0, 255), 2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            RC.send_velocity(0, 0)
            RC.send('e')
            return False, robot_head
        cv2.imshow('Display', display)
        # plt.plot(controller.e_k_list)         # plotting graph of error
        # plt.show()

    if state is True:
        RC.send('p')
        cv2.waitKey(2000)
        state = False
        while state is False:
            ret, frame = videoCapture.read()
            if not ret:
                continue
            frame = cv2.flip(frame, -1)

            # tracking the robot
            robot_space = AP.get_robot_space(frame)
            _, warped_arena = AP.arena_preprocess(frame, M)
            _, (x, y), robot_head, heading_angle = RT.robot_track(robot_space)

            (goal_node_x, goal_node_y) = goal_node
            goal_y = int(goal_node_y * BLOCK_LENGTH + 50)
            goal_x = int(goal_node_x * BLOCK_LENGTH + 50)

            temp_goal_x, temp_goal_y = goal_x, goal_y

            # # Accounting for OverShoot
            # if robot_head == 0:
            #     temp_goal_x = goal_x - 10
            # if robot_head == 1:
            #     temp_goal_y = goal_y + 10
            # if robot_head == 2:
            #     temp_goal_x = goal_x + 10
            # if robot_head == 3:
            #     temp_goal_y = goal_y - 10

            (v_l, v_r), theta_goal, state = controller.capture_object((x, y), heading_angle, (temp_goal_x, temp_goal_y))

            RC.send_velocity(v_l, v_r)

            # add important info to the display
            display = RT.show_robot(warped_arena, (x, y), heading_angle)
            cv2.circle(display, (goal_x, goal_y), 20, (0, 0, 0), 3)
            cv2.putText(display, 'GOTO', (10, 30), 1, 2, (0, 0, 255), 2)
            cv2.putText(display, 'v_L:' + str(v_l) + ' v_R:' + str(v_r), (10, 60), 1, 2, (255, 0, 255), 2)
            cv2.putText(display, 'theta: ' + str(math.degrees(heading_angle)), (10, 90), 1, 2, (255, 0, 255), 2)
            cv2.putText(display, 'theta_goal: ' + str(math.degrees(theta_goal)), (10, 120), 1, 2, (255, 0, 255), 2)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                RC.send_velocity(0, 0)
                RC.send('e')
                return False, robot_head
            cv2.imshow('Display', display)
            # plt.plot(controller.e_k_list)         # plotting graph of error
            # plt.show()

    return True
