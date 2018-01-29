"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         PathPlanner.py
* Theme:            Launch A Module
* Functions:        a_star_search. path_to, plan_path, show_path
* Global Variables: None
"""

import heapq as priorityq
import random

import Grid
import cv2

"""
* Function Name: a_star_search
* Input: grid, start, goal
* Output: came_from - gives previous node from which it came in the arena
* Logic: uses the a* algorithm to find what nodes the path must travel through
* Example Call: parents = a_star_search(grid_handler, start, goal)
"""


def a_star_search(grid_handler, start, goal):
    grid = Grid.GridHandler(grid_handler)
    frontier = []
    priorityq.heappush(frontier, (0, start))
    came_from = {start: None}  # to check if a node is visited and if so its parent
    cost_so_far = {start: 0}  # to hold the cost to reach a node from start

    goal_found = False
    while len(frontier) > 0:
        current = priorityq.heappop(frontier)[1]  # node is visited
        if current == goal:  # Early Exit: Goal found
            goal_found = True
            break
        for neighbor in grid.neighbors(current, goal):
            new_cost = cost_so_far[current] + grid.movement_cost(came_from[current],
                                                                 neighbor)  # constant movement cost is added
            if neighbor not in came_from or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + grid.heuristic(goal, neighbor)
                priorityq.heappush(frontier, (priority, neighbor))  # add neighbor to the frontier
                came_from[neighbor] = current  # the new element of frontier is visited and comes from current

    if goal_found:
        return came_from
    else:
        return None


"""
* Function Name: path_to
* Input: parents (nodes), start, goal
* Output: path from start to goal
* Logic: using parents from a* search finds the path between start and goal
* Example Call: path = path_to(parents, start, goal)
"""


def path_to(parents, start, goal):
    path = [goal]
    node = goal
    while parents[node] != start:
        path.append(parents[node])
        node = parents[node]
    path.append(start)
    path.reverse()  # start -> goal ordering
    return path


"""
* Function Name: plan_path
* Input: to_node, grid_handler
* Output: path
* Logic: calls a* search to find a path and gets the path from path_to function
* Example Call: path = plan_path(to_node, grid_handler)
"""


def plan_path(robot_node, to_node, grid_handler):
    grid = Grid.GridHandler(grid_handler)
    parents = a_star_search(grid, robot_node, to_node)
    if parents:
        path = path_to(parents, robot_node, to_node)
        return path
    else:
        return None


"""
* Function Name: show_path
* Input: arena, path
* Output: display with path
* Logic: draws lines on display based on path
* Example Call: disp = show_path(arena, path)
"""


def show_path(arena, path):
    disp = arena.copy()
    L = 100
    (x, y) = path[0]
    prev_pix = (x * L + int(.5 * L), y * L + int(.5 * L))
    cv2.circle(disp, prev_pix, 10, (130, 0, 75), thickness=2)
    component = lambda: random.randint(0, 255)
    color = (component(), component(), component())
    for point in path:
        (x, y) = point
        pix = (x * L + int(.5 * L), y * L + int(.5 * L))
        cv2.line(disp, pix, prev_pix, color, 2)
        prev_pix = pix
    cv2.circle(disp, pix, 5, (255, 199, 0), thickness=2)
    return disp
