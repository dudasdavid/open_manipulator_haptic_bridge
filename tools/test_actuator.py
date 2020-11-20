#!/usr/bin/python

import sys
import rospy
from open_manipulator_msgs.srv import *

def set_actuator_state(state):
    service_name = '/open_manipulator/set_actuator_state'

    rospy.wait_for_service(service_name)

    try:
        set_state = rospy.ServiceProxy(service_name, SetActuatorState)

        arg = SetActuatorStateRequest()
        if state.lower() == "true":
            arg.set_actuator_state = True
        else:
            arg.set_actuator_state = False
        resp1 = set_state(arg)
        print 'Service done!'
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def usage():
    return "%s [state]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 2:
        state = (sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting [%s]"%(state)
    response = set_actuator_state(state)
    print "[%s] returns [%s]"%(state, response)