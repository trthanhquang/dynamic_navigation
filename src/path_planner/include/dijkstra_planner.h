#ifndef DIJKSTRA_PLANNER
#define DIJKSTRA_PLANNER

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"
#include <cstdlib>
#include <queue>
#include <stdio.h>
#include <limits.h>
#include <map>
#include <vector>
#include <set>
#include <utility>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"

#define mp make_pair
#define pb push_back

#define K_MAX 2.0 // max turning radius = 0.5m
#define V_MAX 1.6 // (m/s)
#define GRID_RESOLUTION 0.1 // (m)

using namespace std;
typedef pair<int, int> pii;
typedef pair<int, pii> pi3;

struct Pose2D{
    double x;
    double y;
    double theta;
};

class DijkstraPlanner{
public:
    DijkstraPlanner(){
    }
    void setCurrentTime(ros::Time now);
    void setCurrentPose(Pose2D current_pose);
    void setGoalPose(Pose2D goal_pose);
    void setStaticMap(boost::shared_ptr<nav_msgs::OccupancyGrid> occup_grid);
    void setObstaclesMap(boost::shared_ptr<nav_msgs::OccupancyGrid> occup_grid);
    void setObstacleList(boost::shared_ptr<path_planner::PoseVelArray> obs_list);
    nav_msgs::Path findPath(void);
    
    vector<Pose2D> searchNextPoses(Pose2D pose, double output_size, double vel, double dt);

private:
    ros::Time current_time;
    set<pii> blocked;
    pii xrange, yrange;
    pii initial_position, terminus;
    double iy,ty;
    double map_resolution;
};

vector<Pose2D> DijkstraPlanner::searchNextPoses(Pose2D cur_pose, double output_size, double vel, double dt){
    vector<Pose2D> nexts;
    double ds = vel*dt;

    cout << "current pose: " << cur_pose.x <<", " << cur_pose.y << ", " << cur_pose.theta << endl;
    if(output_size>1){
        double dk = (2.0*K_MAX)/(output_size-1);
        for(int i=0;i<output_size;i++){
            double k = -K_MAX + dk*i;
            double dtheta = k*ds;
            double new_theta = cur_pose.theta + dtheta;
            double dx = ds*sin(new_theta);
            double dy = ds*cos(new_theta);
            
            //wrap yaw in [0..2pi)
            while (new_theta >= 2.0*M_PI){
                new_theta -= 2.0*M_PI;
            }
            while (new_theta < 0){
                new_theta += 2.0*M_PI;
            }

            Pose2D next;
            next.x = cur_pose.x + dx;
            next.y = cur_pose.y + dy;
            next.theta = new_theta;

            cout << "next pose: " << next.x <<", " << next.y << ", " << next.theta << endl;
            nexts.push_back(next);
        }
    }

    return nexts;
}

void DijkstraPlanner::setCurrentTime(ros::Time now){
    current_time = now;
}

void DijkstraPlanner::setCurrentPose(Pose2D current_pose){
    int x1 = current_pose.x/GRID_RESOLUTION;
    int y1 = current_pose.y/GRID_RESOLUTION;
    this->initial_position = mp(x1,y1);
    this->iy = current_pose.theta;
    std::cout<<"Init: " << x1 <<", "<< y1 <<", " << current_pose.theta <<std::endl;
}

void DijkstraPlanner::setGoalPose(Pose2D goal_pose){
    int x1 = goal_pose.x/GRID_RESOLUTION;
    int y1 = goal_pose.y/GRID_RESOLUTION;
    this->terminus = mp(x1,y1);
    this->ty = goal_pose.theta;
    std::cout<<"Goal: " << x1 <<", "<< y1 <<", " << goal_pose.theta <<std::endl;
}

void DijkstraPlanner::setStaticMap(boost::shared_ptr<nav_msgs::OccupancyGrid> static_grid){
    int width = static_grid->info.width;
    int height = static_grid->info.height;
    
    this->xrange=mp(1,width);
    this->yrange=mp(1,height);

    for(int yi=0; yi<width; yi++)
        for(int xi=0; xi<height; xi++)
            if(static_grid->data[xi+yi*width] > 0)
            {
                for(int a=max(0,xi-4);a<min(height,xi+5);a++)
                    for(int b=max(0,yi-4);b<min(width,yi+5);b++)
                        this->blocked.insert(std::make_pair(a,b));
            }
}

void DijkstraPlanner::setObstacleList(boost::shared_ptr<path_planner::PoseVelArray> obs_list){
    std::cout << "Detect Obstacles: " << obs_list->data.size() << std::endl;
    for(int i=0; i<obs_list->data.size(); i++){
        std::cout << "obstacle " << i 
                  << ", x=" << obs_list->data[i].pose.position.x 
                  << ", y=" << obs_list->data[i].pose.position.y 
                  << ", yaw=" << tf::getYaw(obs_list->data[i].pose.orientation)
                  << ", vel=" << obs_list->data[i].vel
                  << std::endl;
    }
}

void DijkstraPlanner::setObstaclesMap(boost::shared_ptr<nav_msgs::OccupancyGrid> obs_grid){
}

nav_msgs::Path DijkstraPlanner::findPath(){
    //declarations
    priority_queue<pi3, vector<pi3>, greater<pi3> > Q;
    map<pii, int> dist;
    map<pii, pii> dad;
 
    pii initial_position=this->initial_position,terminus=this->terminus,xrange=this->xrange,yrange=this->yrange;
    set<pii>blocked =this->blocked;
    //initializations
    Q.push(mp(0, initial_position));
    dist[initial_position] = 0;

    //Djikstra
    while (!Q.empty()) {
        pi3 p = Q.top();
        if ((p.second).first == terminus.first && (p.second).second == terminus.second) break;
        Q.pop();

        pii here = p.second;
        for (int i = 1; i < 9; i++) {
            int dx = (i - (i % 3)) / 3, dy = i % 3;
            pii nxt = mp((here.first) + 1 - dx, (here.second) + 1 - dy);
            if (blocked.find(nxt) != blocked.end() || nxt.first < xrange.first || nxt.first > xrange.second || nxt.second < yrange.first || nxt.second > yrange.second)continue;

            if (dist.find(nxt) == dist.end() || dist[here] + 1 < dist[nxt]) {
                dist[nxt] = dist[here] + 1;
                dad[nxt] = here;
                Q.push(mp(dist[nxt], nxt));
            }

        }
    }
   
    vector<vector<double> > path_plan;
    vector<double> tpose;
    tpose.pb(terminus.first);
    tpose.pb(terminus.second);
    tpose.pb(this->ty);
    path_plan.pb(tpose);

    double new_yaw;
    double yaws[9]={0,M_PI/2,3*M_PI/2,0,M_PI/4,7*M_PI/4,M_PI,3*M_PI/4,5*M_PI/4};
    if (dist.find(terminus) != dist.end())
        for (pii i = terminus; dad.find(i) != dad.end(); i = dad[i]){
        vector<double> new_pose,pose=path_plan.back();
        new_yaw=yaws[int(3*(abs(pose[0]-i.first)+(pose[0]<i.first))+abs(pose[1]-i.second)+(pose[1]<i.second))];
        if(new_yaw!=pose[2]){
            vector<double> new_pose_turn;
            new_pose_turn.pb(pose[0]);
                    new_pose_turn.pb(pose[1]);
                    new_pose_turn.pb(new_yaw);
            path_plan.pb(new_pose_turn);
        }
        new_pose.pb(i.first);
        new_pose.pb(i.second);
        new_pose.pb(new_yaw);
        path_plan.pb(new_pose);
    }
    vector<geometry_msgs::PoseStamped> path_points;
    double ts=0;
    for(vector<vector<double> >::reverse_iterator it=path_plan.rbegin();it!=path_plan.rend();it++){
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.stamp = current_time+ros::Duration(ts);ts+=0.2;
        string frame = "/map";
        current_pose.header.frame_id = frame.c_str();
        vector<double> point3=*it;

        current_pose.pose.position.x=point3[0]*GRID_RESOLUTION;
        current_pose.pose.position.y=point3[1]*GRID_RESOLUTION;

        geometry_msgs::Quaternion yawor=tf::createQuaternionMsgFromYaw(point3[2]);;
        //tf.quaternionTFToMsg(qtn,yawor);
        current_pose.pose.orientation = yawor;
        path_points.pb(current_pose);
    }

    nav_msgs::Path path_final;
    path_final.poses.resize(path_points.size());
    path_final.header.frame_id = path_points[0].header.frame_id;
    path_final.header.stamp = path_points[0].header.stamp;

    for(int i =0;i < path_points.size();i++)
        path_final.poses[i]=path_points[i];

    return path_final;
}

#endif
