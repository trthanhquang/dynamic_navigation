#!/usr/bin/env python  
import rospy
import tf
import numpy as np

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from path_planner.msg import PoseVel, PoseVelArray

from sets import Set

obstacles_list = [
    {
        'id': 'obstacle 1',
        't': [0, 1,  6,  7, 12, 13],
        'x': [3, 3, 15, 15,  3,  3],
        'y': [6, 6,  6,  6,  6,  6]
    },
    {
        'id': 'obstacle 2',
        't': [ 0, 1,   5,  7, 12, 13],
        'x': [14, 14, 14, 14, 14, 14],
        'y': [ 8, 8,  15, 15,  8,  8]
    }    
]

SENSORS_RADIUS = 6
current_pose = None

def generate_vel_profile():
    for obstacle in obstacles_list:
        t_profile = np.arange(0, obstacle['t'][-1], 0.1)
        x = np.interp(t_profile, obstacle['t'], obstacle['x'])
        y = np.interp(t_profile, obstacle['t'], obstacle['y'])

        dx = np.diff(x)
        dy = np.diff(y)
        dxy = (dx**2 + dy**2)**0.5
        dt = np.diff(t_profile)

        v_profile = dxy/dt
        t_profile = t_profile[:-1]

        obstacle['v_profile']={ 't':t_profile, 'v':v_profile }

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

def poseCallback(msg):
    global current_pose
    current_pose = msg

if __name__ == '__main__':
    rospy.init_node('obstacle_broadcaster')
    obs_markers_pub = rospy.Publisher('obstacle_markers',MarkerArray, queue_size=10)
    occupacy_pub = rospy.Publisher('obstacle_cost_map',OccupancyGrid, queue_size=10)
    obs_posevel_pub = rospy.Publisher('obstacle_posevel_array',PoseVelArray, queue_size=10)
    pose_sub = rospy.Subscriber('current_pose', PoseStamped , poseCallback)

    generate_vel_profile()

    static_map = get_static_map()
    print static_map.info

    r = rospy.Rate(50)
    tStart = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        time_passed = rospy.Time.now().to_sec() - tStart

        obs_pv_arrays = PoseVelArray()
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

            v = np.interp(t, obstacle['v_profile']['t'], obstacle['v_profile']['v'])

            if current_pose!=None:
                dx = m.pose.position.x - current_pose.pose.position.x
                dy = m.pose.position.y - current_pose.pose.position.y
                d = (dx**2 + dy**2)**0.5
                
                if d < SENSORS_RADIUS:

                    pv = PoseVel()
                    pv.pose.position.x = m.pose.position.x
                    pv.pose.position.y = m.pose.position.y
                    pv.pose.orientation.w = m.pose.orientation.w
                    pv.vel = v
                    
                    obs_pv_arrays.data.append(pv)

        # publishing msgs
        occupacy_pub.publish(obs_map)
        obs_markers_pub.publish(ma)
        obs_posevel_pub.publish(obs_pv_arrays)
        r.sleep()
    
