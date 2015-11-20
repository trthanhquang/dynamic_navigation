import rospy
from nav_msgs.srv import * 
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,Quaternion
from tf.transformations import quaternion_from_euler

import numpy as np

goal = None
if len(sys.argv)==4:
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])
    goal = (x,y,yaw)
else:
    print 'syntax: python dstar.py x y yaw'
    sys.exit(1)

rospy.init_node("dstar")

#Init Static Map
rospy.wait_for_service('static_map')
get_map = rospy.ServiceProxy('static_map', GetMap)
static_map = get_map().map
map_width = static_map.info.width
map_height = static_map.info.height 

#Subscriber for Obstacle Map
obs_map = None
current_pose = None
def obs_map_callback(msg):
    global obs_map
    obs_map = msg

def poseCallback(msg):
    global current_pose
    current_pose = msg

obsSub = rospy.Subscriber("obstacle_cost_map",OccupancyGrid, obs_map_callback)
pose_sub = rospy.Subscriber('current_pose', PoseStamped , poseCallback)
pathPub = rospy.Publisher("global_path", Path, queue_size=10)

#---------------------------------------------------------------
# GENERAL HELPING FUNCTIONs
def combine_with_static(obs_map):
    global static_map
    new_grid = np.zeros(len(static_map.data))

    for x in range(map_width):
        for y in range(map_height):
            s = x+y*map_width
            if static_map.data[s]>0 or (obs_map!=None and obs_map.data[s]>0):
                for a in range(max(0,x-5), min(map_width, x+5)):
                    for b in range(max(0,y-5), min(map_height, y+5)):
                        new_grid[a+b*map_width] = 1
    return new_grid

def generate_path_msg(path):
    pathmsg = Path()
    pathmsg.header.frame_id = "map"

    start_t = rospy.Time.now()
    length = 0
    for x,y,g in path:
        ps = PoseStamped()
        ps.header.frame_id = "map"

        if len(pathmsg.poses)>0:
            length += ((pathmsg.poses[-1].pose.position.x - x*0.1)**2+
                (pathmsg.poses[-1].pose.position.y - y*0.1)**2)**0.5

        ps.header.stamp = start_t+rospy.Duration(length/1.6)
        
        position = [x*0.1,y*0.1,0]
        theta = 0
        orientation = quaternion_from_euler(0,0,theta)
        ps.pose.position = Point(*position)
        ps.pose.orientation = Quaternion(*orientation)
        pathmsg.poses.append(ps)

    return pathmsg

def pos_to_s(pos):
    return pos[0]+pos[1]*map_width

def s_to_pos(index):
    x = index%map_width
    y = (index-x)/map_width
    return (x,y)

def succ_s(s):
    pos = s_to_pos(s)
    neighbors = [
         [pos[0]+1,pos[1]],
         [pos[0]-1,pos[1]],
         [pos[0]  ,pos[1]+1],
         [pos[0]  ,pos[1]-1],
         [pos[0]+1,pos[1]+1],
         [pos[0]-1,pos[1]+1],
         [pos[0]+1,pos[1]-1],
         [pos[0]-1,pos[1]-1],
        ]
    res_l = []
    for xy in neighbors:
        if xy[0]<0 or xy[0]>=map_width or xy[1]<0 or xy[1]>=map_height: #within map
            continue
        s = pos_to_s(xy)
        res_l.append(s)
    return res_l

def pred_s(s):
    return succ_s(s)

def h(s1, s2):
    pos1 = s_to_pos(s1)
    pos2 = s_to_pos(s2)
    return abs(pos1[0]-pos2[0])+abs(pos1[1]-pos2[1])

def cost(s,s2):
    global map_data
    if map_data[s]>0 or map_data[s2]>0:
        return float('inf')
    else:
        if not s2 in succ_s(s):
            print 's, s2 are not next to one another'
        return 1
        # pos1 = s_to_pos(s)
        # pos2 = s_to_pos(s2)

        # return ((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)**0.5

#---------------------------------------------------------------
# DSTAR SPECIFIC 
def key(s):
    global g,rhs,start_s
    # return [ min(g[s], rhs[s]) + h(start_s,s), min(g[s], rhs[s])]
    
    esp = 1
    if g[s]>rhs[s]:
        return [rhs[s] + esp*h(start_s,s), rhs[s]]
    else:
        return [g[s]+ h(start_s,s), g[s]]
        
def updateState(s):
    global g, rhs, goal_s, OPEN, OPEN_KEY
    
    if visited[s]==0:
        g[s]= float('inf')
    visited[s]=1
    
    if s!=goal_s:
        rhs[s] = min([cost(s,s2)+g[s2] for s2 in succ_s(s)])
        
    if s in OPEN:
        idx = OPEN.index(s)
        del OPEN[idx]
        del OPEN_KEY[idx]
        
    if g[s]!=rhs[s]:
        OPEN.append(s)
        OPEN_KEY.append(key(s))

def computeShortestPath():
    global OPEN_KEY, OPEN, start_s, rhs, g
    
    cnt = 0
    while min(OPEN_KEY)<key(start_s) or rhs[start_s]!=g[start_s]:        
        min_k = min(OPEN_KEY)
        idx = OPEN_KEY.index(min_k)
        s = OPEN[idx]
        del OPEN[idx]
        del OPEN_KEY[idx]
        
        if g[s]>rhs[s]:
            g[s]=rhs[s]
            for s2 in pred_s(s):
                updateState(s2)
                cnt +=1
        else:
            g[s]=float('inf')
            for s2 in pred_s(s):
                updateState(s2)
                cnt+=1
            updateState(s)
            cnt+=1
    print 'updateState %s times'%cnt

def computePathFromG():
    global g, start_s
    path = []

    print 'g[goal_s]=',g[goal_s],', rhs[goal]=',rhs[goal_s]
    g[goal_s] = 0
    s = start_s
    while g[s]>0:
        [min_g,min_s] = min([[g[si],si] for si in succ_s(s)])
        if min_g>0 and min_g >= g[s]:
            print "unable to find path to goal" 
            break
        else:
            if map_data[s]==0:
                s = min_s
                pos = s_to_pos(s)
                path.append([pos[0], pos[1], g[s]])
            else:
                print "path cross obstacle"
                break
    return path

#---------------------------------------------------------------
# MAIN

while current_pose==None:
    print 'waiting for init pose'
print 'init pose received'

visited = np.zeros(map_width*map_height)
g = np.ones(map_width*map_height)*float('inf') #cost to goal
rhs = np.ones(map_width*map_height)*float('inf') 

# start_pos = (20,20)
start_pos = (current_pose.pose.position.x/0.1,current_pose.pose.position.y/0.1)
goal_pos  = (goal[0]/0.1, goal[1]/0.1)
start_s = pos_to_s(start_pos)
goal_s = pos_to_s(goal_pos)

g[start_s]= float('inf')
rhs[start_s]=float('inf')
g[goal_s]=float('inf')
rhs[goal_s]=0

OPEN = [goal_s]
OPEN_KEY = [key(goal_s)]

map_data = combine_with_static(obs_map)

computeShortestPath()
path = computePathFromG()
print 'Path:',path
msg = generate_path_msg(path)
pathPub.publish(msg)


r = rospy.Rate(10)
while not rospy.is_shutdown():
    visited = np.zeros(map_width*map_height)
    g = np.ones(map_width*map_height)*float('inf') #cost to goal
    rhs = np.ones(map_width*map_height)*float('inf') 

    start_pos = (current_pose.pose.position.x/0.1,current_pose.pose.position.y/0.1)
    start_s = pos_to_s(start_pos)
    goal_s = pos_to_s(goal_pos)

    g[start_s]= float('inf')
    rhs[start_s]=float('inf')
    g[goal_s]=float('inf')
    rhs[goal_s]=0

    OPEN = [goal_s]
    OPEN_KEY = [key(goal_s)]
    map_data = combine_with_static(obs_map)

    # prev_map_data = map_data
    # map_data = combine_with_static(obs_map)
    # for s in range(len(map_data)):
    #     if map_data[s]!= prev_map_data[s]:
    #         # print 'find out changes at',s_to_pos(s),'updateState!'
    #         updateState(s)
    #         for si in succ_s(s):
    #             # print '--> update succ',s_to_pos(si)
    #             updateState(si)

    computeShortestPath()
    path = computePathFromG()
    print 'Path:',path
    msg = generate_path_msg(path)
    pathPub.publish(msg)
    r.sleep()