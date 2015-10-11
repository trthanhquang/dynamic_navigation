#!/usr/bin/env python  
import rospy
import tf
import numpy as np 

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Pose

posePub = None
global_path = None
vel = 1 

def path_callback(msg):
	global global_path
	global_path = msg
    

def start_sim():
    global posePub
    r = rospy.Rate(10)
    
    pose_list = global_path.poses 
    for pose in pose_list:
        pose.header.frame_id = 'map'        
        posePub.publish(pose)
        print 'set pose: ',pose.pose.position.x,pose.pose.position.y
        r.sleep()

if __name__ == '__main__':
    global posePub
    rospy.init_node('actuator')
    rospy.Subscriber("global_path",Path,path_callback)
    posePub = rospy.Publisher('current_pose', PoseStamped, queue_size=10);
    
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        if global_path!=None:
            start_sim()
        r.sleep()