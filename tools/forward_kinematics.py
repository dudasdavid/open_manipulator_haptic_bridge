#!/usr/bin/python3

import math

offset_x = 0.012
offset_z = 0.0595 + 0.017
l1 = 0.128
l2 = 0.024
l3 = 0.124
l4 = 0.126


def forward_kinematics(joints):
    pos = [0,0,0]

    x = offset_x + (l1 * math.sin(joints[1]) + l2 * math.cos(joints[1]) + l3 * math.cos(joints[1] + joints[2]) + l4 * math.cos(joints[1] + joints[2] + joints[3])) * math.cos(joints[0])
    y = (l1 * math.sin(joints[1]) + l2 * math.cos(joints[1]) + l3 * math.cos(joints[1] + joints[2]) + l4 * math.cos(joints[1] + joints[2] + joints[3])) * math.sin(joints[0])
    z = offset_z + l1 * math.cos(joints[1]) - l2 * math.sin(joints[1]) - l3 * math.sin(joints[1] + joints[2]) - l4 * math.sin(joints[1] + joints[2] + joints[3])

    pos[0] = x
    pos[1] = y
    pos[2] = z
    return pos



if __name__ == "__main__":
    while 1:
        input_text = input("Joint angles separated by commas, type 'exit' to break: ")
        if input_text == 'exit':
            break
        elif len(input_text.split(',')) == 4:
            try:
                angles = []
                for angle in input_text.split(','):
                    angles.append(float(angle))

                print(f"Joint angles: {angles}")
                pos = forward_kinematics(angles)
                print(f"End effector position: {pos}")
            except:
                print("Invalid input: %s" % input_text)
        else:
            print("Invalid input: %s" % input_text)