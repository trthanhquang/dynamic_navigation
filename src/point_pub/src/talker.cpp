#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Path>("global_path", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    nav_msgs::Path msg;
    msg.header.frame_id="map";
    int cnt=0;
    while(cnt++<10){
      geometry_msgs::PoseStamped ps;
      
      ps.pose.orientation.x=0;
      ps.pose.orientation.y=0;
      ps.pose.orientation.z=0;
      ps.pose.orientation.w=0;

      ps.pose.position.x=cnt+0.5;
      ps.pose.position.y=cnt+0.5;
      ps.pose.position.z=0;

      msg.poses.push_back(ps);
    }
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    // break;
  }

  return 0;
}
