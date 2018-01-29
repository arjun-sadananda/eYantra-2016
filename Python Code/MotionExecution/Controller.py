"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         Controller.py
* Theme:            Launch A Module
* Functions:        go_to_goal, hard_turn_to_theta
* Global Variables: V_MAX, V_MIN, WHEEL_DIAMETER, TRACK, dt
"""

import math

V_MAX = 255
V_MIN = 110
WHEEL_DIAMETER = 51.0
TRACK = 150.0

dt = .1


'''
* Class name: Controller
* Description: holds history of errors and uses PID control to achieve path
'''

class Controller:
    def __init__(self):
        self.e_k_i = 0.0
        self.e_k_1 = 0.0

        self.e_k_list = []

    """
    * Function Name:    go_to_goal
    * Input:            pos_x, pos_y, heading, goal_x, goal_y
    * Output:           left_velocity, right_velocity, completion state
    * Logic:            Uses PID control to continuously set velocities
                        based on history of error from actual path
    * Example Call:     go_to_goal((robot_x, robot_y), heading, (goal_x, goal_y))
    """
    def go_to_goal(self, (pos_x, pos_y), heading, (goal_x, goal_y)):

        Kp = 1000   # 5000
        Ki = 300    # 1500
        Kd = 800    # 4000

        v = 175

        GOAL_RADIUS_ALLOWANCE = 20  #20
        u_y = goal_y - pos_y
        u_x = goal_x - pos_x

        dist_from_goal = math.sqrt(u_y * u_y + u_x * u_x)

        if dist_from_goal < 15 * GOAL_RADIUS_ALLOWANCE:
            v = 200 - (15 * GOAL_RADIUS_ALLOWANCE - dist_from_goal) * .4

        if dist_from_goal < GOAL_RADIUS_ALLOWANCE:
            return (0, 0), 0,  True

        theta_goal = math.atan2(u_y, u_x)

        e_k = heading - theta_goal

        e_k = math.atan2(math.sin(e_k), math.cos(e_k))

        # for steering
        self.e_k_list.append(e_k)

        e_P = e_k

        e_I = self.e_k_i + e_k * dt

        e_D = (e_k - self.e_k_1) / dt

        w = Kp * e_P + Ki * e_I + Kd * e_D

        self.e_k_i = e_I

        self.e_k_1 = e_k

        delta = 0
        # unicycle to differential drive
        v_r = (v + w) - delta
        v_l = (v - w) + delta

        if v_r > V_MAX:
            v_r = V_MAX

        elif v_r < V_MIN:
            v_r = V_MIN     # -V_MIN - (V_MIN - v_r)

        if v_l > V_MAX:
            v_l = V_MAX

        elif v_l < V_MIN:
            v_l = V_MIN     # -V_MIN - (V_MIN - v_l)

        # last parameter is reachedDest
        return (v_l, v_r), theta_goal, False

    # FOR SOFT TURNS
    # def go_to_theta(self, robot_heading, desired_heading):
    #
    #     Kp = 10
    #     Ki = 0
    #     Kd = .1
    #
    #     v = 175
    #
    #     THETA_ALLOWANCE = math.pi / 12
    #
    #     e_theta = desired_heading - robot_heading
    #
    #     # if (e_theta < THETA_ALLOWANCE * 4):
    #     #     # if (abs(u_y) < GOAL_RADIUS_ALLOWANCE & & abs(u_x) < GOAL_RADIUS_ALLOWANCE){
    #     #     v = 200 - (4 * THETA_ALLOWANCE - e_theta) * 20
    #
    #     if abs(e_theta) < THETA_ALLOWANCE:
    #         return (0, 0),  True
    #
    #     e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))
    #
    #     # for steering
    #
    #     e_P = e_theta
    #
    #     e_I = self.e_k_i + e_theta * dt
    #
    #     e_D = (e_theta - self.e_k_1) / dt
    #
    #     w = Kp * e_P + Ki * e_I + Kd * e_D
    #
    #     self.e_k_i = e_I
    #
    #     self.e_k_1 = e_theta
    #
    #     # unicycle to differential drive
    #     delta = 10
    #     v_r = (v + w)
    #     v_l = (v - w) + delta
    #
    #     if v_r > V_MAX:
    #         v_r = V_MAX
    #
    #     elif v_r < V_MIN:
    #         v_r = V_MIN     # -V_MIN - (V_MIN - v_r)
    #
    #     if v_l > V_MAX:
    #         v_l = V_MAX
    #
    #     elif v_l < V_MIN:
    #         v_l = V_MIN     # -V_MIN - (V_MIN - v_l)
    #
    #     # last parameter is reachedDest
    #     return (v_l, v_r), False

    """
        * Function Name:    hard_turn_to_theta
        * Input:            pos_x, pos_y, heading, goal_x, goal_y, rotation, turn
        * Output:           left_velocity, right_velocity, completion state
        * Logic:            Uses PID control to continuously set velocities
                            based on history of error from actual angle
        * Example Call:      hard_turn_to_theta((pos_x, pos_y), heading, (goal_x, goal_y), rotation, turn)
        """

    def hard_turn_to_theta(self, (pos_x, pos_y), heading, (goal_x, goal_y), rotation, turn): # rotation = 0, 90, 180
        if rotation == 0: # 0 deg allignment
            print'alligning'
            GOAL_THETA_ALLOWANCE = math.pi / 36
            Kp = 2500  # 2000  alternate tuning values
            Ki = 1000  # 1250
            Kd = 2000
        elif rotation == 2: # 180 deg turns
            print'rotate 180'
            GOAL_THETA_ALLOWANCE = math.pi / 6
            Kp = 4000
            Ki = 0  # 1250
            Kd = 1050 #1000
        else:   # 90 deg turn
            print 'rotate 90'
            GOAL_THETA_ALLOWANCE = math.pi / 12
            Kp = 3000   #2000
            Ki = 0
            Kd = 1000   #1000

        v = 0

        u_y = goal_y - pos_y
        u_x = goal_x - pos_x

        theta_goal = math.atan2(u_y, u_x)

        e_k = heading - theta_goal
        e_k = math.atan2(math.sin(e_k), math.cos(e_k))

        if turn == 'R':
            if e_k > 0:
                e_k -= 2 * math.pi
        if turn == 'L':
            if e_k < 0:
                e_k += 2 * math.pi

        if (abs(e_k) < GOAL_THETA_ALLOWANCE):
            # if (abs(u_y) < GOAL_RADIUS_ALLOWANCE & & abs(u_x) < GOAL_RADIUS_ALLOWANCE){
            return (0, 0), 0,  True

        # for steering
        self.e_k_list.append(e_k)

        e_P = e_k

        e_I = self.e_k_i + e_k * dt

        e_D = (e_k - self.e_k_1) / dt

        w = Kp * e_P + Ki * e_I + Kd * e_D

        # print e_P, e_I, e_D

        self.e_k_i = e_I

        self.e_k_1 = e_k

        delta = 0
        # unicycle to differential drive
        v_r = (v + w) / 30
        v_l = (v - w) / 30

        # ensure it is within the limit range
        if abs(v_r) > V_MAX:
            v_r = V_MAX * v_r/abs(v_r)

        elif abs(v_r) < V_MIN:
            v_r = V_MIN * v_r/abs(v_r)     # -V_MIN - (V_MIN - v_r)

        if abs(v_l) > V_MAX:
            v_l = V_MAX * v_l/abs(v_l)

        elif abs(v_l) < V_MIN:
            v_l = V_MIN  * v_l/abs(v_l)   # -V_MIN - (V_MIN - v_l)

        # last parameter is reachedDest
        return (v_l, v_r), theta_goal, False

    def capture_object(self, (pos_x, pos_y), heading, (goal_x, goal_y)):
        Kp = 2000
        Ki = 700
        Kd = 2000

        v = 100

        GOAL_RADIUS_ALLOWANCE = 20  # 20
        u_y = goal_y - pos_y
        u_x = goal_x - pos_x

        dist_from_goal = math.sqrt(u_y * u_y + u_x * u_x)

        if dist_from_goal < 15 * GOAL_RADIUS_ALLOWANCE:
            v = 200 - (15 * GOAL_RADIUS_ALLOWANCE - dist_from_goal) * .4

        if dist_from_goal < GOAL_RADIUS_ALLOWANCE:
            return (0, 0), 0, True

        theta_goal = math.atan2(u_y, u_x)

        e_k = heading - theta_goal

        e_k = math.atan2(math.sin(e_k), math.cos(e_k))

        # for steering
        self.e_k_list.append(e_k)

        e_P = e_k

        e_I = self.e_k_i + e_k * dt

        e_D = (e_k - self.e_k_1) / dt

        w = Kp * e_P + Ki * e_I + Kd * e_D

        self.e_k_i = e_I

        self.e_k_1 = e_k

        delta = 0
        # unicycle to differential drive
        v_r = (v + w) - delta
        v_l = (v - w) + delta

        if v_r > V_MAX:
            v_r = V_MAX

        elif v_r < V_MIN:
            v_r = V_MIN  # -V_MIN - (V_MIN - v_r)

        if v_l > V_MAX:
            v_l = V_MAX

        elif v_l < V_MIN:
            v_l = V_MIN  # -V_MIN - (V_MIN - v_l)

        # last parameter is reachedDest
        return (v_l, v_r), theta_goal, False
