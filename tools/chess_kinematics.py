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

    x = coords[0] - offset_x# - l4 * math.cos(j0)
    y = coords[1]# - l4 * math.sin(j0)
    z = coords[2] - offset_z + l4

    # recalculate x
    x = math.sqrt(y*y + x*x)

    c = math.sqrt(x*x + z*z)
    alpha = math.asin(z/c)
    beta = (math.pi / 2.0) - alpha

    gamma = math.acos((l1c*l1c + c*c - l3*l3)/(2*c*l1c))

    j1 = (math.pi / 2.0) - alpha - gamma - j1o
    j2 = math.acos((l1c*l1c + l3*l3 - c*c)/(2*l1c*l3)) - j2o
    
    delta = math.pi - gamma - j2 - j2o

    j3 = math.pi - (beta + delta)


    angles[0] = j0
    angles[1] = j1
    angles[2] = -j2
    angles[3] = j3
    return angles

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

    coords = [0.1,0.1,0.15]

    angles = inverse_kinematics(coords)
    pos = forward_kinematics(angles)

    print(coords)
    print(pos)
    print(angles)

    joints = [0.8491414759301353, 0.07432307757203777, -0.7101448753353337, 2.2066181245581924]
    pos = forward_kinematics(joints)
    angles = inverse_kinematics(pos)

    print(joints)
    print(angles)
    print(pos)
