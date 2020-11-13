#!/usr/bin/python

import sys
import rospy
from open_manipulator_msgs.srv import *

def set_joint_client(gripper, time):
    service_name = '/open_manipulator/goal_tool_control'

    rospy.wait_for_service(service_name)

    try:
        set_joint_angles = rospy.ServiceProxy(service_name, SetJointPosition)

        arg = SetJointPositionRequest()
        arg.joint_position.joint_name = ["gripper"]
        arg.joint_position.position = [gripper]
        arg.path_time = time
        resp1 = set_joint_angles(arg)
        print 'Service done!'
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def usage():
    return "%s [gripper time]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 3:
        gripper = float(sys.argv[1])
        time = float(sys.argv[2])

    else:
        print usage()
        sys.exit(1)
    print "Requesting angles [%s]"%(gripper)
    response = set_joint_client(gripper, time)
    print "[%s] returns [%s]"%(gripper, response)