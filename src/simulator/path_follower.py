#!/usr/bin/env python  
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np 

def path_to_list(path):
    tL = []
    xL = []
    yL = []
    yawL = []
    for ps in path.poses:
        tL.append(ps.header.stamp.to_sec())
        xL.append(ps.pose.position.x)
        yL.append(ps.pose.position.y)

        q = ( ps.pose.orientation.x, ps.pose.orientation.y, 
            ps.pose.orientation.z, ps.pose.orientation.w )
        rpy = euler_from_quaternion(q)
        yaw = rpy[2] % (2*np.pi)
        
        if len(yawL)>0:
            while (yaw - yawL[-1]) >= 2*np.pi:
                yaw -= 2*np.pi
            while yaw - yawL[-1] <= -2*np.pi:
                yaw += 2*np.pi
        yawL.append(yaw)
        
    return tL,xL,yL,yawL

def path_callback(msg):
    global global_path
    global_path = path_to_list(msg)

if __name__ == '__main__':
    global posePub, global_path
    global_path = None

    rospy.init_node("path_follower")
    rospy.Subscriber("global_path",Path,path_callback)
    posePub = rospy.Publisher("current_pose", PoseStamped, queue_size=10)
    
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        if global_path!=None:
            tL,xL,yL,yawL = global_path

            t = rospy.Time.now().to_sec()
            x = np.interp(t,tL,xL)
            y = np.interp(t,tL,yL)
            yaw = np.interp(t,tL,yawL)

            ps = PoseStamped()
            ps.header.stamp = rospy.Time.from_sec(t)
            ps.pose.position = Point(*([x,y,0]))
        
            q = quaternion_from_euler(0,0,yaw)
            ps.pose.orientation = Quaternion(*q)
            
            posePub.publish(ps)

        r.sleep()