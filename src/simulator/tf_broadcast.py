#!/usr/bin/env python  
import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped,Pose, Quaternion
import math

def set_robot_pose(pose):
    br = tf.TransformBroadcaster()
    position = (pose.position.x, pose.position.y, pose.position.z)
    orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    br.sendTransform(position, orientation, rospy.Time.now(),'base_link','map')

def position_callback(msg):
    global curPose
    curPose = msg.pose

if __name__ == '__main__':
    global curPose
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber("current_pose",PoseStamped,position_callback)
    
    curPose = Pose()
    curPose.position.x = 2
    curPose.position.y = 2

    curPose.orientation = Quaternion(*quaternion_from_euler(0,0,math.pi/4))
    
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        set_robot_pose(curPose)
        r.sleep()