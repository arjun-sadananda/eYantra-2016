"""
* Team Id :         LM#4787
* Author List :     Arjun S, Vinod, Arvind, Vishnu
* Filename:         RobotCommunication.py
* Theme:            Launch A Module
* Functions:        start, send, send_velocity
* Global Variables: None
"""

import serial

"""
* Function Name: start
* Input: -
* Output: -
* Logic: establishes communication with robot
* Example Call: start()
"""

def start():
    global robot
    PORT = "/dev/ttyUSB0"  # change the port if you are not using Windows to whatever port you are using
    BAUD_RATE = 9600
    robot = serial.Serial(PORT, BAUD_RATE, timeout=1)
    return

"""
* Function Name: send
* Input: i (raw data)
* Output: data is sent to robot
* Logic: sends encoded data to robot
* Example Call: send(65)
"""

def send(i):
    data = i  # raw_input()
    robot.write(data.encode())  # if you are using python 3 replace data with data.encode()

    # except KeyboardInterrupt:
    #     robot.close()
    #     break
    return


"""
* Function Name: send_velocity
* Input: v_l, v_r
* Output: wheel velocities are sent to robot
* Logic: sends encoded data to robot using send function
* Example Call: send_velocity(-100, 100)
"""


def send_velocity(v_l, v_r):

    # decide the sign
    if v_l > 0:
        signv_l = '+'
    else:
        signv_l = '-'
    if v_r > 0:
        signv_r = '+'
    else:
        signv_r = '-'

    strv_l = str(abs(int(v_l)))
    strv_r = str(abs(int(v_r)))

    # ensure there are 3 digits
    while len(strv_l) < 3:
        strv_l = '0' + strv_l
    while len(strv_r) < 3:
        strv_r = '0' + strv_r

    data = 'g' + signv_l + strv_l + signv_r + strv_r
    robot.write(data.encode())

    return
