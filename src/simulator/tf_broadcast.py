#!/usr/bin/env python  
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform((1,1,0), tf.transformations.quaternion_from_euler(0, 0,0), rospy.Time.now(),'base_link','map')
        #print 'sending tf base_link/map'
        r.sleep()
    
