#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "dijkstra_planner.h"

//some transform API documentaion: http://docs.ros.org/api/tf/html/c++/transform__datatypes_8h.html
#include "tf/transform_datatypes.h" 

#include <vector>

bool is_map_initialized = false;
bool is_pose_initialized = false;

nav_msgs::OccupancyGrid global_map;
geometry_msgs::PoseStamped current_pose;

void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    is_map_initialized = true;
    global_map = msg;
}

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
    is_pose_initialized = true;
    current_pose = msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;
    ros::Subscriber mapSub  = n.subscribe("map", 1, mapCallback);
    ros::Subscriber poseSub = n.subscribe("current_pose", 1, poseCallback);
    ros::Publisher path_pub =  n.advertise<nav_msgs::Path>("global_path",1000);

    ros::Rate loop_rate(10);
    std::cout << "Initializing..." << std::endl;
    while(ros::ok()){
        if (is_map_initialized && is_pose_initialized){
            std::cout << "Current Pose: x " << current_pose.pose.position.x
                      << ", y " << current_pose.pose.position.y
                      << ", theta(rad) " << tf::getYaw(current_pose.pose.orientation) <<std::endl;

            std::vector<int8_t> data = global_map.data;
            std::cout << "Map Info: width " << global_map.info.width 
                      << ", height " << global_map.info.height
                      << ", data vector size: " << global_map.data.size() 
                      << std::endl;

            DijkstraPlanner solver;
            solver.setInit(current_pose.pose.position.x, current_pose.pose.position.y, 
                tf::getYaw(current_pose.pose.orientation));
            // solver.setInit(2, 2, 3.14/4.0);
            solver.setGoal(1, 12.5, 3.14);
            solver.setMap(global_map);

            std::cout<< "solving path..." <<std::endl;
            nav_msgs::Path path = solver.solve_path();
            std::cout<< "path len: " << path.poses.size() << std::endl;
            path_pub.publish(path);
            std::cout<< "---------" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}