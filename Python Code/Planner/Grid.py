"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         Grid.py
* Theme:            Launch A Module
* Functions:        copy, heuristic, math_goal_to_objects, show_matches, show_robot, show_objects
*                   movement_cost, is_allowed, neighbours
* Global Variables: None
"""

import cv2
import random

'''
* Class name: GridHandler
* Description: holds grid, objects, robot and obstacles information and related methods
'''


class GridHandler:
    def __init__(self, orig=None):
        if orig is None:
            self.sideX = 9
            self.sideY = 6
            self.goals = {}  # {(0, y): [color, shape, size]
            self.objects = {}  # {(x, y): [color, shape, size]}
            self.obstacles = []  # [(x, y)]
            self.matches = {}  # {(0, goalY): (objectX, objectY)}
            self.occupied = []

        else:
            self.copy(orig)

    """
    * Function Name:    copy
    * Input:            GridHandler object
    * Output:           Matching values in original object
    * Logic:            assigns values from object to self
    * Example Call:     self.copy(new_obj)
    """

    def copy(self, orig):
        self.sideX = 9
        self.sideY = 6
        self.goals = orig.goals
        self.objects = orig.objects
        self.obstacles = orig.obstacles
        self.matches = orig.matches
        self.occupied = orig.occupied

    """
    * Function Name: heuristic
    * Input: node_a, node_b
    * Output: distance
    * Logic: returns manhattan distance between node_a and node_b
    * Example Call: man_dist = heuristic(node_a, node_b)
    """

    @staticmethod
    def heuristic(node_a, node_b):
        (x1, y1) = node_a
        (x2, y2) = node_b
        return abs(x1 - x2) + abs(y1 - y2)

    """
    * Function Name:   match_goal_to_object
    * Input:           -
    * Output:          matches stored in self
    * Logic:           makes use of linear search to find which goal matches which object
                      chooses that object which is minimum distance from goal
    * Example Call:    match_goal_to_object()
    """

    @property
    def match_goal_to_object(self):
        matches_made = 0
        obj = []  # list of matching objects for a goal
        man_dist = []  # respective manhattan distance between object and goal

        self.occupied = self.obstacles + list(self.objects.keys())  # list of occupied nodes

        for goal_node in self.goals.keys():
            (color, shape, size) = self.goals.get(goal_node)

            for object_node in self.objects.keys():
                if self.objects.get(object_node)[0] == color and \
                                self.objects.get(object_node)[1] == shape and \
                                self.objects.get(object_node)[2] == size:
                    obj.append(object_node)
                    man_dist.append(self.heuristic(object_node, goal_node))
            if len(obj) == 1:
                self.matches[goal_node] = obj[0]
                matches_made += 1
                # self.occupied.append(obj[0])
                del self.objects[obj[0]]
            elif len(obj) > 1:
                # if more than one object match the goal: choose one with smaller distance
                # final_object_node = obj[man_dist.index(min(man_dist))]
                final_object_node = obj[0]
                self.matches[goal_node] = final_object_node
                matches_made += 1
                # self.occupied.append(final_object_node)
                del self.objects[final_object_node]

            else:
                print 'No Match'

            del obj[:]

        return matches_made


    """
    * Function Name:  show_objects
    * Input:           arena
    * Output:          modified display with match info
    * Logic:           displays details about objects on display such as color
    * Example Call:    show_objects(arena)
    """

    def show_objects(self, arena):
        disp = arena.copy()
        for node in self.obstacles:
            cv2.circle(disp, (100 * node[0] + 50, 100 * node[1] + 50), 40, (0, 0, 0), 2)
        for node in self.objects.keys():
            if self.objects.get(node)[0] == 'Red':
                cv2.circle(disp, (100 * node[0] + 50, 100 * node[1] + 50), 40, (0, 0, 255), 2)
            if self.objects.get(node)[0] == 'Blue':
                cv2.circle(disp, (100 * node[0] + 50, 100 * node[1] + 50), 40, (255, 0, 0), 2)
            if self.objects.get(node)[0] == 'Green':
                cv2.circle(disp, (100 * node[0] + 50, 100 * node[1] + 50), 40, (0, 255, 0), 2)
            # cv2.putText(disp, )
        return disp

    """
    * Function Name:  show_matches
    * Input:           arena
    * Output:          modified display with match info
    * Logic:           displays details about matches between goals and objects on display
    * Example Call:    show_matches(arena)
    """

    def show_matches(self, arena):
        disp = arena.copy()
        for goal_node in self.matches.keys():
            # To pick random color
            component = lambda: random.randint(0, 255)
            color = (component(), component(), component())
            cv2.circle(disp, (50, 100 * goal_node[1] + 50), 30, color, 2)
            (x, y) = self.matches.get(goal_node)
            cv2.circle(disp, (100 * x + 50, 100 * y + 50), 30, color, 2)
        return disp

    """
    * Function Name:   show_robot
    * Input:           arena
    * Output:          modified display with robot info
    * Logic:           displays details about robot on display such as position and heading
    * Example Call:    show_robot(arena)
    """

    # def show_robot(self, arena):
    #     disp = arena.copy()
    #     (x, y) = self.robot_node
    #     pos_body = (100 * x + 50, 100 * y + 50)
    #     cv2.circle(disp, pos_body, 20, (255, 0, 255), 3)
    #     cv2.ellipse(disp, pos_body, (20, 20), 90 * self.robot_head, -30, 30, (130, 0, 75), 4)
    #     cv2.ellipse(disp, pos_body, (30, 30), 90 * self.robot_head, -30, 30, (112, 25, 25), 4)
    #     cv2.ellipse(disp, pos_body, (40, 40), 90 * self.robot_head, -30, 30, (128, 0, 0), 4)
    #     cv2.ellipse(disp, pos_body, (50, 50), 90 * self.robot_head, -30, 30, (225, 105, 65), 3)
    #     cv2.ellipse(disp, pos_body, (60, 60), 90 * self.robot_head, -30, 30, (255, 191, 0), 2)
    #
    #     return disp

    """
    * Function Name: movement_cost
    * Input: prev_node, next_node
    * Output: constant(weight)
    * Logic: return weight of movement between tiles
    * Example Call: cost = movement_cost(node_prev, node_next)
    """

    @staticmethod
    def movement_cost(prev_node, to_node):
        if prev_node:
            (prevX, prevY) = prev_node
            (toX, toY) = to_node
            if toX != prevX and toY != prevY:
                return 1.5
            # Straight Line motion has lower movement cost
            elif toX == prevX or toY == prevY:
                return 1
            # Making turns has higher movement cost
            else:
                return 2
        return 1

    # To avoid turning towards it while making hard turns
    def is_obstacle(self, node):
        return node in self.obstacles

    """
    * Function Name: is_allowed
    * Input: node
    * Output: Boolean:True/False
    * Logic: checks if node->(x,y) is in bounds(1 to side) and not occupied
    * Example Call: is_allowed(node)
    """

    def is_allowed(self, node):
        (x, y) = node
        return 1 <= x <= self.sideX - 1 and 0 <= y <= self.sideY - 1 and (node not in self.occupied)

    """
    * Function Name: neighbors
    * Input: node, goal
    * Output: list of neighboring nodes that are allowed
    * Logic: create list of all (up, left, down, right: vertical and horizontal only) neighbors
                and filter out neighbors that are not allowed
                (Since goal is "occupied", goal s explicitly added to list of it is a neighbor)
    * Example Call: neighbors = neighbors(node, goal)
    """

    def neighbors(self, node, goal):
        (x, y) = node
        results = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        diag_nodes = []
        if self.is_allowed(results[0]) and self.is_allowed(results[1]):
            diag_nodes.append((x+1, y-1))
        if self.is_allowed(results[1]) and self.is_allowed(results[2]):
            diag_nodes.append((x-1, y-1))
        if self.is_allowed(results[2]) and self.is_allowed(results[3]):
            diag_nodes.append((x-1, y+1))
        if self.is_allowed(results[3]) and self.is_allowed(results[0]):
            diag_nodes.append((x+1, y+1))
        diag_nodes = filter(self.is_allowed, diag_nodes)

        if goal in results:  # filter will filter out the goal position, therefore append goal
            results = filter(self.is_allowed, results)
            results.append(goal)
        else:
            results = filter(self.is_allowed, results)

        return results + diag_nodes
