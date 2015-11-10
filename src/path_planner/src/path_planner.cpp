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
//ros_services
#include "nav_msgs/GetMap.h" 

//ros libs
#include "tf/transform_datatypes.h"  //http://docs.ros.org/api/tf/html/c++/transform__datatypes_8h.html

//planner libs
#include "dijkstra_planner.h"
#include <vector>

bool is_pose_initialized = false;
bool anytime_replanning = true;

boost::shared_ptr<nav_msgs::OccupancyGrid> global_obstacle_map;
boost::shared_ptr<nav_msgs::OccupancyGrid> global_static_map;
geometry_msgs::PoseStamped current_pose;

double goal_x = 1;
double goal_y = 12.5;
double goal_yaw = 3.14;

void obstacleMapCallback(boost::shared_ptr<nav_msgs::OccupancyGrid> msg){
    global_obstacle_map = msg;
}

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
    is_pose_initialized = true;
    current_pose = msg;
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

    }    
    std::cout << "Initializing ROS..." << std::endl;

    ros::Subscriber poseSub = n.subscribe("current_pose", 1, poseCallback);
    ros::Subscriber obsMapSub = n.subscribe("obstacle_cost_map", 1, obstacleMapCallback);
    ros::Publisher pathPub =  n.advertise<nav_msgs::Path>("global_path",1);
    
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

    while(ros::ok()){
        if (is_pose_initialized){
            std::cout << "Current Pose: x " << current_pose.pose.position.x
                      << ", y " << current_pose.pose.position.y
                      << ", theta(rad) " << tf::getYaw(current_pose.pose.orientation) <<std::endl;

            if(global_obstacle_map){
                std::cout << "Obstacle Map received. width " << global_obstacle_map->info.width 
                  << ", height " << global_obstacle_map->info.height
                  << ", resolution " << global_obstacle_map->info.resolution
                  << ", data length: " << global_obstacle_map->data.size() 
                  << std::endl;
            }

            DijkstraPlanner solver;
            solver.setInitTime(ros::Time::now());
            solver.setInitPose(current_pose.pose.position.x, current_pose.pose.position.y, 
                               tf::getYaw(current_pose.pose.orientation));
            solver.setGoalPose(goal_x, goal_y, goal_yaw);
            solver.setStaticMap(global_static_map);
            if(global_obstacle_map){
                solver.setObstaclesMap(global_obstacle_map);
            }

            std::cout<< "[AnytimePlanning] solving path..." <<std::endl;
            path = solver.findPath();   
            std::cout<< "path len: " << path.poses.size() << std::endl;

            pathPub.publish(path);
            std::cout<< "---------" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}