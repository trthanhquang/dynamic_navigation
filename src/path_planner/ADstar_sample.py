import rospy
import numpy as np

from nav_msgs.srv import * 
from nav_msgs.msg import OccupancyGrid
rospy.init_node("ipython")
rospy.wait_for_service('static_map')
get_map = rospy.ServiceProxy('static_map', GetMap)
static_map = get_map().map

obs_map = None
def obs_map_callback(msg):
    global obs_map
    obs_map = msg

sub = rospy.Subscriber("obstacle_cost_map",OccupancyGrid, obs_map_callback)

# May have to wait a while here before the obs_map is ready

map_width = static_map.info.width
map_height = static_map.info.height
map_data = static_map.data
obs_data = []
if obs_map:
    obs_data = obs_map.data

# map_width = 6
# map_height = 7
# map_data = [
#     1, 1, 1, 1, 1, 0,
#     0, 0, 0, 0, 1, 0,
#     0, 1, 1, 0, 1, 0,
#     0, 1, 0, 0, 0, 0,
#     0, 1, 0, 0, 0, 0,
#     0, 1, 1, 1, 1, 0,
#     0, 0, 0, 0, 0, 0
# ]

## Combine obs_map and static_map + inflation for robot
combine_data = np.zeros(len(map_data))
for x in range(map_width):
    for y in range(map_height):
        s = x+y*map_width
        if map_data[s]>0 or obs_data[s]>0:
            for a in range(max(0,x-5), min(map_width, x+5)):
                for b in range(max(0,y-5), min(map_height, y+5)):
                    combine_data[a+b*map_width] = 1

# General helping functions

def pos_to_index(pos):
    return pos[0]+pos[1]*map_width

def index_to_pos(index):
    x = index%map_width
    y = (index-x)/map_width
    return (x,y)

def h(pos1, pos2):
    return abs(pos1[0]-pos2[0])+abs(pos1[1]-pos2[1])

def cost(pos1, pos2):
    global combine_data
    i1 = pos_to_index(pos1)
    i2 = pos_to_index(pos2)
    if combine_data[i1]==0 and combine_data[i2]==0:
        return ((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)**0.5
    else:
        return float('inf')

def succ_index(index):
    pos = index_to_pos(index)
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
    succ_l = []
    for xy in neighbors:
        if xy[0]<0 or xy[0]>=map_width or xy[1]<0 or xy[1]>=map_height: #within map
            continue
        s = pos_to_index(xy)
        if map_data[s]==0: # does not hit obstacle
            succ_l.append(s)
    return succ_l

def pred_index(index):
    return succ_index(index)

# ADStar specific functions

open_l = []
open_l_key = []
open_min_k = float('inf')

closed_l = []
incons_l = []

g = np.ones(map_width*map_height)*float('inf') #cost to goal
rhs = np.zeros(map_width*map_height)

def key(s):
    if g[s] > rhs[s]:
        return [rhs[s] + esp*h(start_pos,index_to_pos(s)), rhs[s] ]
    else:
        return [g[s]   +     h(start_pos,index_to_pos(s)), g[s]   ]

def UpdateState(s):
    global open_l, incons_l, rhs, g, open_l_key, open_min_k
    
    if s!=goal_index:
        current_pos = index_to_pos(s)
        
        #rhs[s] = min succ{c+g}
        min_cost = float('inf')
        for si in succ_index(s):
            next_pos = index_to_pos(si)
            rhs_tmp = cost(current_pos,next_pos)+g[si]
            if rhs_tmp < min_cost:
                min_cost = rhs_tmp
        rhs[s]= min_cost
    
    if s in open_l:
        idx = open_l.index(s)
        del open_l[idx]
        del open_l_key[idx]
        
    if g[s]!=rhs[s]:
        if s not in closed_l:
            open_l.append(s)
            open_l_key.append(key(s))
        else:
            incons_l.append(s)
    
def ComputeorImprovePath():
    global open_l, open_l_key, open_min_k,incons_l, rhs, g
    cnt = 0

    open_min_k = min(open_l_key)
    while ((open_min_k < key(start_index)) or (rhs[start_index]!=g[start_index])):
        #remove state s with minimum key from OPEN
        idx = open_l_key.index(open_min_k)
        s = open_l[idx]
        del open_l[idx]
        del open_l_key[idx]
        
        if g[s]>rhs[s]:
            g[s]=rhs[s]
            closed_l.append(s)
            for s1 in pred_index(s):
                UpdateState(s1)
                cnt+=1
        else:
            g[s]=float('inf')
            for s1 in pred_index(s)+[s]:
                UpdateState(s1)
                cnt+=1
        
        open_min_k = min(open_l_key)

    print 'UpdateState cnt: ',cnt


start_pos = (10,10)
goal_pos  = (20,180)

start_index = pos_to_index(start_pos)
goal_index = pos_to_index(goal_pos)
esp = 1.0

g[start_index]=float('inf')
rhs[start_index]=float('inf')
g[goal_index]=float('inf')
rhs[goal_index]=0

# Start of the algorithm

open_l = [goal_index]
open_l_key = [key(goal_index)]
open_min_k = 0
ComputeorImprovePath()


# Transfer from inconsistent list to open list

print len(incons_l)
print len(open_l)
for s in incons_l:
    open_l.append(s)
    open_l_key.append(key(s))

incons_l = []
closed_l = []
esp = 1
ComputeorImprovePath()


# create path by following g

path = []
s = start_index
path.append(index_to_pos(s))

while g[s]>0:
    [min_k,min_s] = min([[g[si],si] for si in succ_index(s)])
    s = min_s
    pos = index_to_pos(s)
    path.append(pos)


