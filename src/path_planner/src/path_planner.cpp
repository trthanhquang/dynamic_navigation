#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>

bool is_initialized = false;
nav_msgs::OccupancyGrid map;
geometry_msgs::PoseStamped current_pose;

void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    is_initialized = true;
    map = msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("map", 1, mapCallback);

    ros::Rate loop_rate(1);
    while(ros::ok()){
        if (is_initialized){
            std::cout << map.header <<std::endl;
            std::cout << map.info <<std::endl;
            
            std::cout << "Width: " << map.info.width <<std::endl;
            std::cout << "Heigh: " << map.info.height <<std::endl;

            std::vector<int8_t> data = map.data;
            std::cout << data.size() <<std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}