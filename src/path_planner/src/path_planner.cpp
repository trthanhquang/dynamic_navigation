/*
@Author: Louis Tran (trthanhquang@gmail.com)

Path planner node get information from environment (static & dynamic obstacles)
and plan a path for robot to navigate from initial to goal destination

Subcribe/Services:
    - static_map (OccupancyGrid): get static map from map_server via service call
    - current_pose (PoseStamped): current position & orientation of the robot
    - obstacle_cost_map (OccupancyGrid): published by simulator/obstacles.py
        grid cells which have obstacles are marked with 100, empty is 0
    
    Potential topics:
    - obstacle_trackers : lists of obstacles estimation (state & var). 
        each state is pose and velocity vector

Publish:
    - global_path (Path): time stamp and pose. 
        This path is taken by simulator/path_follower.py. Path follower interpolates waypoints
        given in this global_path and just assume global_path is valid which means the path
        satisfies kinodynamic constraints (i.e non-holonomic constraints, speed constraints)

How to run:
    roslaunch <repo>/robot_and_map.launch
    1. rosrun path_planner path_planner_node
    2. rosrun path_planner path_planner_node goal_x goal_y goal_yaw_rad
*/

#include "ros/ros.h"
//ros_msgs
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "path_planner/PoseVel.h"
#include "path_planner/PoseVelArray.h"
//ros_services
#include "nav_msgs/GetMap.h" 

//ros libs
#include "tf/transform_datatypes.h"  //http://docs.ros.org/api/tf/html/c++/transform__datatypes_8h.html

//planner libs
#include "dijkstra_replanner.h"
#include <vector>

bool is_pose_initialized = false;
bool is_goal_initialized = false;
bool anytime_replanning = true;

boost::shared_ptr<nav_msgs::OccupancyGrid> global_obstacle_map;
boost::shared_ptr<nav_msgs::OccupancyGrid> global_static_map;
boost::shared_ptr<path_planner::PoseVelArray> obstacle_posevel_list;

geometry_msgs::PoseStamped current_pose;

double goal_x = 0;
double goal_y = 0;
double goal_yaw = 0;

void obstacleMapCallback(boost::shared_ptr<nav_msgs::OccupancyGrid> msg){
    global_obstacle_map = msg;
}

void obsPoseVelCallback(boost::shared_ptr<path_planner::PoseVelArray> msg){
    obstacle_posevel_list = msg;
}

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
    is_pose_initialized = true;
    current_pose = msg;
}

void goalPoseCallback(const geometry_msgs::Pose& msg)
{
    is_goal_initialized = true;
    goal_x = msg.position.x;
    goal_y = msg.position.y;
    goal_yaw = tf::getYaw(msg.orientation);

    std::cout << "Receiving new goal: x=" <<goal_x
              << ", y=" << goal_y <<", yaw=" << goal_yaw 
              << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;

    if(argc>3){
        goal_x = std::strtod(argv[1],NULL);
        goal_y = std::strtod(argv[2],NULL);
        goal_yaw = std::strtod(argv[3],NULL);
        std::cout << "Goal configuration: x=" <<goal_x
                  << ", y=" << goal_y <<", yaw=" << goal_yaw 
                  << std::endl;
        is_goal_initialized= true;
    }
    std::cout << "Initializing ROS..." << std::endl;

    ros::Subscriber poseSub = n.subscribe("current_pose", 1, poseCallback);
    ros::Subscriber navigationGoalSub = n.subscribe("goal_pose", 1, goalPoseCallback);
    ros::Subscriber obsMapSub = n.subscribe("obstacle_cost_map", 1, obstacleMapCallback);
    ros::Subscriber obsPoseVelSub = n.subscribe("obstacle_posevel_array", 1, obsPoseVelCallback);
    ros::Publisher pathPub =  n.advertise<nav_msgs::Path>("global_path",10);
    
    //Get map from map_server
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;
    if (client.call(srv)){
        global_static_map.reset(new nav_msgs::OccupancyGrid(srv.response.map));

        std::cout << "Static Map received: width " << global_static_map->info.width 
                  << ", height " << global_static_map->info.height
                  << ", resolution " << global_static_map->info.resolution
                  << ", data length: " << global_static_map->data.size() 
                  << std::endl; 
    } else{
        ROS_ERROR("Failed to get map! Terminate program!");
        return 1;
    }

    ros::Rate loop_rate(10);
    nav_msgs::Path path;

    DijkstraReplanner solver;

    ros::Subscriber obs_sub = n.subscribe("obstacle_cost_map", 1, &DijkstraReplanner::setObstacleMap, &solver);

    solver.setStaticMap(global_static_map);

    std::cout << "Waiting for initial Pose and goal Pose to start" << std::endl;
    
    while(ros::ok()){
        if (is_pose_initialized and is_goal_initialized){
            std::cout << "Current Pose: x " << current_pose.pose.position.x
                      << ", y " << current_pose.pose.position.y
                      << ", theta(rad) " << tf::getYaw(current_pose.pose.orientation) <<std::endl;
            
            Pose2D start_2dpose, goal_2dpose;

            if (path.poses.size() >=2){
                start_2dpose.x = current_pose.pose.position.x;
                start_2dpose.y = current_pose.pose.position.y;
                start_2dpose.theta = tf::getYaw(current_pose.pose.orientation);                
            }else{
                start_2dpose.x = path.poses[1].pose.position.x;
                start_2dpose.y = path.poses[1].pose.position.y;
                start_2dpose.theta = tf::getYaw(path.poses[1].pose.orientation);
            }

            goal_2dpose.x = goal_x;
            goal_2dpose.y = goal_y;
            goal_2dpose.theta = goal_yaw;

            solver.searchNextPoses(start_2dpose, 5, 1.6, 0.2);

            solver.setCurrentTime(ros::Time::now());
            solver.setCurrentPose(start_2dpose);
            solver.setGoalPose(goal_2dpose);
            
            if(obstacle_posevel_list)
                solver.setObstacleList(obstacle_posevel_list);

            std::cout<< "[AnytimePlanning] solving path..." <<std::endl;
            path = solver.findPath();
            std::cout<< "path len: " << path.poses.size() << std::endl;

            if(path.poses.size() < 2){
                std::cout<< "Invalid path! output path need to have at least 2 waypoints" << std::endl;
            }else{
                pathPub.publish(path);
                std::cout<< "---------" << std::endl;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}