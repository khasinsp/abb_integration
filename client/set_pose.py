import socket
import numpy as np
from struct import pack, unpack

FREQ = 100
ANGLE_THRESHOLD = 10
SPEED_THRESHOLD = 10

def generate_motion(command: list):

    speed = command.pop(-1)
    max_angle = max(command)
    commandlen = max_angle / speed * FREQ

    send_command = np.array([np.linspace(0, command[0], commandlen),
                             np.linspace(0, command[1], commandlen),
                             np.linspace(0, command[2], commandlen),
                             np.linspace(0, command[3], commandlen),
                             np.linspace(0, command[4], commandlen),
                             np.linspace(0, command[5], commandlen)])
    return send_command.T


def main():

    while True:
    
        command = input("angle joint1, angle joint2, angle joint3, angle joint4, angle joint5, angle joint6, angular_speed")
        command = command.split(",")
        command = [float(c) for c in command]

        high_angle = False
        high_speed = False

        for c, i in enumerate(command):
            if i < len(command)-1:
                if c > ANGLE_THRESHOLD:
                    print(f"angle too big for joint {i+1}")
                    high_angle = True
            if i == len(command)-1:
                if c > SPEED_THRESHOLD:
                    print(f"Speed too big: {c}")
                    high_speed = True

            
        if high_angle or high_speed:
            print("Invalid values, try again!")
            continue
        
        command_arr = generate_motion(command)

        for c in command_arr:
            command = []
            command.append(c.tolist())
            




