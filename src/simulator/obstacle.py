#!/usr/bin/env python  
import rospy
import tf
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('obstacle_broadcaster')
    pub = rospy.Publisher('/viz_marker',Marker, queue_size=10)
    r = rospy.Rate(10)
    
    m = Marker()
    m.header.frame_id='map'
    m.scale.x=1;
    m.scale.y=1;
    m.scale.z=0.1;
    m.type = Marker.SPHERE
    m.color.a = 1.0

    while not rospy.is_shutdown():
        rx = np.append(np.arange(3,15,0.1), np.arange(15,3,-0.1))
        for xi in rx:
            if rospy.is_shutdown():
                break
            m.pose.position.x=xi
            m.pose.position.y=6
            pub.publish(m)
            r.sleep()
        
