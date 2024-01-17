#!/usr/bin/python3

import rospy
import rosnode
import random
import os

from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.msg import Reference

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        rospy.loginfo('ros not initialized')

        publishers = []
        n_uavs = 1

        ## | --------------------- service clients -------------------- |

        self.sc_path = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)

        path_msg = PathSrvRequest()

        path_msg.path.header.frame_id = ""
        path_msg.path.header.stamp = rospy.Time.now()

        path_msg.path.use_heading = True
        path_msg.path.fly_now = True

        path_msg.path.max_execution_time = 5.0
        path_msg.path.max_deviation_from_path = 0.0
        path_msg.path.dont_prepend_current_state = False

        sign = 1.0

        for i in range(0, 10):

            point = Reference()

            point.position.x = i*2
            point.position.y = sign*0.5
            point.position.z = 5
            point.heading = 0

            sign = sign * -1

            path_msg.path.points.append(point)

        try:
            response = self.sc_path.call(path_msg)
        except:
            rospy.logerr('[SweepingGenerator]: path service not callable')
            pass

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
