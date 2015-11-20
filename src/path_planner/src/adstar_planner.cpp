#include "ros/ros.h"
//ros_msgs
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "path_planner/PoseVel.h"
#include "path_planner/PoseVelArray.h"
#include "geometry_msgs/Pose2D.h"

//ros_services
#include "nav_msgs/GetMap.h" 

//ros libs
#include "tf/transform_datatypes.h"  //http://docs.ros.org/api/tf/html/c++/transform__datatypes_8h.html

//planner libs
#include "ads.h"
#include <vector>

boost::shared_ptr<nav_msgs::OccupancyGrid> global_static_map;

int main(int argc, char **argv){
    ros::init(argc, argv, "path_planner");
    boost::shared_ptr<ros::NodeHandle> nh;
    nh.reset(new ros::NodeHandle);

    Pose2D initPose, goalPose;
    if(argc>6){
        double goal_x, goal_y, goal_theta;
        double init_x, init_y, init_theta;

        init_x = std::strtod(argv[1],NULL);
        init_y = std::strtod(argv[2],NULL);
        init_theta = std::strtod(argv[3],NULL);

        goal_x = std::strtod(argv[4],NULL);
        goal_y = std::strtod(argv[5],NULL);
        goal_theta = std::strtod(argv[6],NULL);
        
        initPose.x = init_x;
        initPose.y = init_y;
        initPose.theta = init_theta;

        goalPose.x = goal_x;
        goalPose.y = goal_y;
        goalPose.theta = goal_theta;
    }else{
        std::cout << "arg[6]: init_x init_y init_theta goal_x goal_y goal_theta" << std::endl;
        return 1;
    }
    std::cout << "Initializing ROS..." << std::endl;
    
    //Get map from map_server
    ros::ServiceClient client = nh->serviceClient<nav_msgs::GetMap>("static_map");
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
    
    AdsPlanner planner;
    planner.setStaticMap(global_static_map);
    planner.setCurrentPose(initPose);
    planner.setGoalPose(goalPose);
    planner.start_planning_node(nh);

    return 0;
}