#!/usr/bin/env python  
import rospy
import tf

from geometry_msgs.msg import PoseStamped,Pose

curPose = Pose()
curPose.position.x = 1 
curPose.position.y = 2

def position_callback(msg):
    global curPose
    curPose = msg.pose

    br = tf.TransformBroadcaster()
    # position = (curPose.position.x, curPose.position.y, curPose.position.z)
    position = (1,2,0)
    orientation = (0, 0, 0, 0)
    br.sendTransform((1,2,0), (0,0,0,0), rospy.Time.now(),'base_link','map')
    print 'hey'

if __name__ == '__main__':
    global curPose
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber("current_pose",PoseStamped,position_callback)

    rospy.spin()