#!/usr/bin/python

import sys
import rospy
from open_manipulator_msgs.srv import *

def set_joint_client(j1, j2, j3, j4, time):
    service_name = '/open_manipulator/goal_joint_space_path'

    rospy.wait_for_service(service_name)

    try:
        set_joint_angles = rospy.ServiceProxy(service_name, SetJointPosition)

        arg = SetJointPositionRequest()
        arg.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        arg.joint_position.position = [j1, j2, j3, j4]
        arg.path_time = time
        resp1 = set_joint_angles(arg)
        print 'Service done!'
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def usage():
    return "%s [j1 j2 j3 j4]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 6:
        j1 = float(sys.argv[1])
        j2 = float(sys.argv[2])
        j3 = float(sys.argv[3])
        j4 = float(sys.argv[4])
        time = float(sys.argv[5])
    else:
        print usage()
        sys.exit(1)
    print "Requesting angles [%s, %s, %s, %s]"%(j1, j2, j3, j4)
    response = set_joint_client(j1, j2, j3, j4, time)
    print "[%s %s %s %s] returns [%s]"%(j1, j2, j3, j4, response)