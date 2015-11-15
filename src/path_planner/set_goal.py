#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Pose,Quaternion
from tf.transformations import quaternion_from_euler

import numpy as np

if __name__ == '__main__':
    if len(sys.argv)==4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])
        print 'setting goal to x=%s(m), y=%s(m), yaw=%s(rad)'%(x,y,yaw)

        rospy.init_node("goal_setter")
        goalPub = rospy.Publisher("goal_pose", Pose, queue_size=10)

        counter = 100

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            p = Pose()
            p.position.x = x
            p.position.y = y
            q = quaternion_from_euler(0,0,yaw)
            p.orientation = Quaternion(*q)

            goalPub.publish(p)
            r.sleep()
            counter-=1
            if counter==0:
                break
    else:
        print 'Wrong syntax: try: ./set_goal.py x y yaw'