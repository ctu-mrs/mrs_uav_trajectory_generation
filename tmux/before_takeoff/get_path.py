#!/usr/bin/python3

import rospy
import rosnode
import random
import os

from mrs_msgs.srv import GetPathSrv,GetPathSrvRequest
from mrs_msgs.msg import Reference

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        rospy.loginfo('ros initialized')

        publishers = []
        n_uavs = 1

        ## | --------------------- service clients -------------------- |

        self.sc_path = rospy.ServiceProxy('/uav1/trajectory_generation/get_path', GetPathSrv)

        path_msg = GetPathSrvRequest()

        path_msg.path.header.frame_id = ""
        path_msg.path.header.stamp = rospy.Time.now()

        path_msg.path.use_heading = True

        sign = 1.0

        for i in range(5, 10):

            for j in range(5, 10):

                point = Reference()

                point.position.x = i
                point.position.y = j
                point.position.z = 5
                point.heading = i

                path_msg.path.points.append(point)

        try:
            response = self.sc_path.call(path_msg)

            print("response: {}".format(response))
        except:
            rospy.logerr('[SweepingGenerator]: path service not callable')
            pass

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
