"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         RobotInstructions.py
* Theme:            Launch A Module
* Functions:        diff_to_dir, path_to_instructions
* Global Variables: None
"""

"""
* Function Name: diff_to_dir
* Input: x, y
* Output: heading direction
* Logic: assigns 0,1,2,3 to different multiples of 90 deg based on difference between 2 nodes
* Example Call: diff_to_dir(0,1)
"""


def diff_to_dir(next_node, robot_node):
    (x, y) = robot_node
    (next_x, next_y) = next_node

    x_diff = next_x - x
    y_diff = next_y - y

    if x_diff == 0 and y_diff >= 1:  # 90
        return 3    # double check...
    elif x_diff <= -1 and y_diff == 0:  # 180
        return 2
    elif x_diff == 0 and y_diff <= -1:  # 270
        return 1
    elif x_diff >= 1 and y_diff == 0:  # 0
        return 0

    elif x_diff >= 1 and y_diff <= -1:  # 270
        return 0.5
    elif x_diff <= -1 and y_diff <= -1:  # 270
        return 1.5
    elif x_diff <= -1 and y_diff >= 1:  # 270
        return 2.5
    elif x_diff >= 1 and y_diff >= 1:  # 270
        return 3.5
    # Add cases 0.5 1.5 2.5 3.5


"""
* Function Name: path_to_instructions
* Input: path, grid_handler (grid details), arena (image)
* Output: the series of instructions which robot can understand
* Logic: finds the relation between next node and current node
*        based on which it appends an 'F','L' or 'R' character
* Example Call: deliver_an_object(grid, display)
"""


def path_to_instructions(robot_node, robot_head, path):
    instructions = {}   # {instruction: [next_node, next_head]}
    F = 0

    goal_node = path[len(path) - 1]

    instruction_num = 0

    for next_node in path:
        if next_node == robot_node:
            continue

        #   FOR SOFT TURNS
        #     desired_head = diff_to_dir(next_node, robot_node)
        #
        #     if desired_head - robot_head != 0:
        #         if desired_head - robot_head == 1 or (desired_head == 0 and robot_head == 3):
        #             instructions['L'] = [robot_node, desired_head]
        #         elif abs(desired_head - robot_head) == 2:
        #             instructions['LL'] = [robot_node, desired_head]
        #         elif desired_head - robot_head == -1 or (desired_head == 3 and robot_head == 0):
        #             instructions.append('R')
        #
        # else:

        print next_node, robot_node
        desired_head = diff_to_dir(next_node, robot_node)

        if next_node == goal_node and desired_head == robot_head: #The Last Move

            (x, y) = robot_node

            if robot_head == 0:
                x += 0.3
            elif robot_head == 1:
                y -= 0.3
            elif robot_head == 2:
                x -= 0.3
            elif robot_head == 3:
                y += 0.3

            instructions[instruction_num] = ['F', (x, y)]
            instructions[instruction_num + 1] = ['T', goal_node]
            # instructions[instruction_num + 2] = ['T', goal_node]
            break

        if desired_head == robot_head:
            F += 1  # Increase Number of Forwards

        if desired_head != robot_head:
            if F > 0:
                (x, y) = robot_node

                if robot_head == 0:
                    x += 0.3
                elif robot_head == 1:
                    y -= 0.3
                elif robot_head == 2:
                    x -= 0.3
                elif robot_head == 3:
                    y += 0.3
                instructions[instruction_num] = ['F', (x, y)]  # First append the Incremented Forwards
                instruction_num += 1
                F = 1
            instructions[instruction_num] = ['T', next_node]
            instruction_num += 1
            F = 1

        # FOR SOFT TURNS
        # elif desired_head - robot_head == 1 or (desired_head == 0 and robot_head == 3):
        #     if F > 0:
        #         instructions[instruction_num] = ['F', robot_node]  # First append the Incremented Forwards
        #         instruction_num += 1
        #         F = 1
        #     instructions[instruction_num] = ['l', robot_node, desired_head]  # Soft Left (Higher than Hard turn Accuracy) and Move to nextNode
        #     instruction_num += 1
        # elif desired_head - robot_head == -1 or (desired_head == 3 and robot_head == 0):
        #     if F > 0:
        #         instructions[instruction_num] = ['F', robot_node]
        #         instruction_num += 1
        #         F = 1
        #     instructions[instruction_num] = ['r', robot_node, desired_head]  # Soft Right and Move to nextNode
        #     instruction_num += 1

        # print 'Desired Head: ', desired_head
        # print 'Robot Head: ', robot_head

        robot_node = next_node
        robot_head = desired_head

    return instructions
