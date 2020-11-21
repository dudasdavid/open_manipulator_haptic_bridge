#!/usr/bin/python3

import math

offset_x = 0.012
offset_z = 0.0595 + 0.017
l1 = 0.128
l2 = 0.024
l1c = 0.13023 # combined l1 - l2 length
l3 = 0.124
l4 = 0.126

j1o = math.atan(l2/l1)
j2o = math.pi - ((math.pi/2.0) - j1o)


def inverse_kinematics(coords):
    angles = [0,0,0,0]

    j0 = math.atan(coords[1]/(coords[0] - offset_x))

    x = coords[0] - offset_x - l4 * math.cos(j0)
    y = coords[1] - l4 * math.sin(j0)
    z = coords[2] - offset_z

    # recalculate x
    x = math.sqrt(y*y + x*x)

    c = math.sqrt(x*x + z*z)
    alpha = math.asin(z/c)
    beta = (math.pi / 2.0) - alpha

    gamma = math.acos((l1c*l1c + c*c - l3*l3)/(2*c*l1c))

    j1 = (math.pi / 2.0) - alpha - gamma - j1o
    j2 = math.acos((l1c*l1c + l3*l3 - c*c)/(2*l1c*l3)) - j2o
    
    delta = math.pi - gamma - j2 - j2o

    j3 = math.pi - (math.pi/2.0 + beta + delta)


    angles[0] = j0
    angles[1] = j1
    angles[2] = -j2
    angles[3] = j3
    return angles



if __name__ == "__main__":
    while 1:
        input_text = input("Task space coordinates separated by commas, type 'exit' to break: ")
        if input_text == 'exit':
            break
        elif len(input_text.split(',')) == 3:
            try:
                coords = []
                for coord in input_text.split(','):
                    coords.append(float(coord))

                print(f"Task space coordinates: {coords}")
                angles = inverse_kinematics(coords)
                print(f"Joint angles: {angles}")
            except:
                print("Invalid input: %s" % input_text)
        else:
            print("Invalid input: %s" % input_text)