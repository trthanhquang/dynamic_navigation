#!/usr/bin/env python  
import rospy
import tf
import numpy as np

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from sets import Set

obstacles_list = [
    {
        'id': 'obstacle 1',
        't': [0,  5,  6, 11, 12],
        'x': [3, 15, 15,  3,  3],
        'y': [6,  6,  6,  6,  6]
    },
    {
        'id': 'obstacle 2',
        't': [0,   5,  6, 11, 12],
        'x': [15, 15, 15, 15, 15],
        'y': [8,  15, 15,  8,  8]
    }    
]

def create_marker(t,x,y, x_size=1, y_size=1, z_size=0.1, m_type=Marker.SPHERE):
    m = Marker()
    m.header.frame_id='map'
    m.scale.x=x_size
    m.scale.y=y_size
    m.scale.z=z_size
    m.type = m_type
    m.color.a = 0.9

    return m

def get_static_map():
    rospy.wait_for_service('static_map')
    try: 
        get_map = rospy.ServiceProxy('static_map', GetMap)
        return get_map().map
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None

def find_cells(x,y,radius,tolerant=0.05,grid_resolution=0.1):
    x_i = int(x/grid_resolution)
    y_i = int(y/grid_resolution)
    r_i = int((radius+tolerant)/grid_resolution)
    
    coords = Set()
    r2 = (r_i)**2
    
    for i in range(r_i+1): # +1 to give full range from [0,r_i]
        for j in range(r_i+1):
            if (i**2 + j**2) <= r2:
                coords.add((x_i + i, y_i + j))
                coords.add((x_i + i, y_i - j))
                coords.add((x_i - i, y_i + j))
                coords.add((x_i - i, y_i - j))
    return coords

if __name__ == '__main__':
    rospy.init_node('obstacle_broadcaster')
    obs_markers_pub = rospy.Publisher('/obstacle_markers',MarkerArray, queue_size=10)
    occupacy_pub = rospy.Publisher('/obstacle_cost_map',OccupancyGrid, queue_size=10)
    
    static_map = get_static_map()
    print static_map.info

    r = rospy.Rate(50)
    tStart = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        time_passed = rospy.Time.now().to_sec() - tStart

        ma = MarkerArray()
        obs_map = OccupancyGrid()
        obs_map.info = static_map.info
        obs_map.header.frame_id = 'map'
        for i in range(obs_map.info.height*obs_map.info.width):
            obs_map.data.append(0)

        for i in range(len(obstacles_list)):
            m = Marker()

            m.header.frame_id='map'
            m.scale.x=1;
            m.scale.y=1;
            m.scale.z=0.1;
            m.type = Marker.SPHERE
            m.color.a = 1.0

            m.id = i
            obstacle = obstacles_list[i]

            t = time_passed % obstacle['t'][-1]
            m.pose.position.x = np.interp(t, obstacle['t'], obstacle['x'])
            m.pose.position.y = np.interp(t, obstacle['t'], obstacle['y'])
            m.pose.orientation.w = 1
            
            ma.markers.append(m)
            
            for xi,yi in find_cells(m.pose.position.x, m.pose.position.y,0.5):
                obs_map.data[xi+yi*obs_map.info.width]=100

        occupacy_pub.publish(obs_map)
        obs_markers_pub.publish(ma)
        r.sleep()
    
