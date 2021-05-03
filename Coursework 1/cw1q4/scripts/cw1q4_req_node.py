#!/usr/bin/env python

import rospy
import random
from cw1q4_srv.srv  import quat2rodrigues
from cw1q4_srv.srv  import quat2rodriguesRequest
from cw1q4_srv.srv  import quat2zyx
from cw1q4_srv.srv  import quat2zyxRequest
from cw1q4_srv.srv  import rotmat2quat
from cw1q4_srv.srv  import rotmat2quatRequest


def rot_convert_client():
    S = 2
    if (S == 1):
        rospy.wait_for_service('quat2rodrigues') 
    elif (S == 2):
        rospy.wait_for_service('rotmat2quat') 
    else:
        rospy.wait_for_service('quat2zyx') 
    
    while not rospy.is_shutdown():
        
        if (S == 1):
            client = rospy.ServiceProxy('quat2rodrigues', quat2rodrigues) #Initialise client for the service "rot_convert"

            req = quat2rodriguesRequest()
            req.q.w = 0.707
            req.q.x = 0.0
            req.q.y = 0.707
            req.q.z = 0.0

        elif (S == 2):
            client = rospy.ServiceProxy('rotmat2quat', rotmat2quat) #Initialise client for the service "rot_convert"

            req = rotmat2quatRequest() #Initialise request message (defined by test_srvRequest).
            
            req.r1.data.append(0)
            req.r1.data.append(0)
            req.r1.data.append(1)

            req.r2.data.append(0)
            req.r2.data.append(1)
            req.r2.data.append(0)

            req.r3.data.append(-1)
            req.r3.data.append(0)
            req.r3.data.append(0)
        else:
            client = rospy.ServiceProxy('quat2zyx', quat2zyx) #Initialise client for the service "rot_convert"

            req = quat2zyxRequest()
            req.q.w = 0.707
            req.q.x = 0.0
            req.q.y = 0.707
            req.q.z = 0.0
        
        resp = client(req) # Get the response from the service.

        print 'The result:'
        if (S == 2):
            print resp.q.w
            print resp.q.x
            print resp.q.y
            print resp.q.z
        else:
            print resp.x
            print resp.y
            print resp.z

if __name__ == "__main__":
    try:
        rot_convert_client()
    except rospy.ROSInterruptException:
        pass
