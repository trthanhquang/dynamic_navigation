#!/usr/bin/env python
import rospy
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
from tf.transformations import quaternion_from_euler

from threading import Thread
import numpy as np

path = Path()
path.header.frame_id = "map"

path_plan = [
    #(t,  x,    y, yaw)
    ( 0,  2,    2, np.pi/4),
    (10,  6,    6, np.pi/4),
    (11,  6,    6, 0      ),
    (20, 12,    6, 0      ),
    (21, 12,    6, np.pi/4),
    (25, 15,    9, np.pi/4),
    (26, 15,    9, np.pi/2),
    (34, 15, 12.5, np.pi/2),
    (35, 15, 12.5, np.pi  ),
    (55,  1, 12.5, np.pi  ),
]

rospy.init_node("path_generator")
pathPub = rospy.Publisher("global_path", Path, queue_size=10)

print 'Start time:',rospy.Time.now()

for t,x,y,theta in path_plan:
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()+rospy.Duration(t)
    ps.header.frame_id = "map"
    
    position = [x,y,0]
    orientation = quaternion_from_euler(0,0,theta)
    ps.pose.position = Point(*position)
    ps.pose.orientation = Quaternion(*orientation)
    path.poses.append(ps)

r = rospy.Rate(5)
while not rospy.is_shutdown():
    pathPub.publish(path)
    r.sleep()
