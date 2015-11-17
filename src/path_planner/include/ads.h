#ifndef ADS_PLANNER
#define ADS_PLANNER

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
#include "geometry_msgs/Pose2D.h"
#include "boost/shared_ptr.hpp"
#include "path_planner/PoseVel.h"
#include "path_planner/PoseVelArray.h"



#define mp make_pair
#define pb push_back

#define K_MAX 2.0 // max turning radius = 0.5m
#define V_MAX 1.6 // (m/s)
#define GRID_RESOLUTION 0.1 // (m)
#define Pose2D geometry_msgs::Pose2D

using namespace std;
typedef pair<int, int> pii;
typedef pair<double, double> pdd;
typedef pair<int, pii> pi3;

class adscomp{
    public:
    bool operator() (const pair<pii ,pdd>& a,const pair<pii ,pdd>& b) const{
            if(a.second.first == b.second.first) return a.second.second > b.second.second;
            else return a.second.first > b.second.first;
    }
};

class AdsPlanner{
public:
    AdsPlanner(){
    }
    void setCurrentTime(ros::Time now);
    void setCurrentPose(Pose2D current_pose);
    void setGoalPose(Pose2D goal_pose);
    void setStaticMap(boost::shared_ptr<nav_msgs::OccupancyGrid> occup_grid);
    void setObstacleMap(boost::shared_ptr<nav_msgs::OccupancyGrid> obs_grid);
    void setObstacleList(boost::shared_ptr<path_planner::PoseVelArray> obs_list);
   // nav_msgs::Path findPath(void);
    pdd key(Pose2D state);
    void updateState(Pose2D state);
    Pose2D makep2d(pii a);
    vector<Pose2D> computeOrImprovePath();
    void start_planning_node(boost::shared_ptr<ros::NodeHandle> nh);
    vector<Pose2D> searchNextPoses(Pose2D pose, double output_size, double vel, double dt);
    pii getpii(Pose2D a);
    void disp2d(Pose2D a);
private:
    ros::Time current_time;
    set<pii> blocked,static_blocked;
    pii xrange, yrange;
    Pose2D initial_position, terminus;
    double iy,ty;
    double map_resolution,epsilon;
    map<pii ,double> cost_to_goal,rhs;
    priority_queue<pair<pii ,pdd>, vector<pair<pii,pdd> >, adscomp > open_mp;
    set<pii> open,closed,incons;
    //map<pii, vector<Pose2D> > dads;
    map<pii, Pose2D> open_st,incons_st;

    boost::shared_ptr<nav_msgs::OccupancyGrid> obstacles_map;
    bool is_obstacle_present;
};

bool lexcomp ( pair<pii ,pdd> a,pair<pii ,pdd> b){
                if(a.second.first == b.second.first) return a.second.second < b.second.second;
                else return a.second.first < b.second.first;
}

/*
vector<Pose2D> AdsPlanner::searchNextPoses(Pose2D cur_pose, double output_size, double vel, double dt){
    vector<Pose2D> nexts;
    double ds = vel*dt;

    //cout << "current pose: " << cur_pose.x <<", " << cur_pose.y << ", " << cur_pose.theta << endl;
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

            //cout << "next pose: " << next.x <<", " << next.y << ", " << next.theta << endl;
            nexts.push_back(next);
	    //this->dads[mp(cur_pose.x,cur_pose.y)].pb(next);
        }
    }

    return nexts;
}
*/

vector<Pose2D> AdsPlanner::searchNextPoses(Pose2D cur_pose, double output_size, double vel, double dt){
double yaws[9]={0,M_PI/2,3*M_PI/2,0,M_PI/4,7*M_PI/4,M_PI,3*M_PI/4,5*M_PI/4};
vector<Pose2D> xx;
for (int i = 0; i < 9; i++) {
Pose2D yy;
        int dx = (i - (i % 3)) / 3, dy = i % 3;
	yy.x= cur_pose.x+GRID_RESOLUTION*(1-dx);
	yy.y=cur_pose.y+GRID_RESOLUTION*(1-dy);
	yy.theta=yaws[int(3*dx+dy)];
	if(i!=4)xx.pb(yy);
}
return xx;
}


void AdsPlanner::setCurrentTime(ros::Time now){
    current_time = now;
}

void AdsPlanner::setCurrentPose(Pose2D current_pose){
    int x1 = current_pose.x/GRID_RESOLUTION;
    int y1 = current_pose.y/GRID_RESOLUTION;
    this->initial_position = current_pose;
    this->iy = current_pose.theta;
    std::cout<<"Init: " << x1 <<", "<< y1 <<", " << current_pose.theta <<std::endl;
}

void AdsPlanner::setGoalPose(Pose2D goal_pose){
    int x1 = goal_pose.x/GRID_RESOLUTION;
    int y1 = goal_pose.y/GRID_RESOLUTION;
    this->terminus = goal_pose;
    this->ty = goal_pose.theta;
    std::cout<<"Goal: " << x1 <<", "<< y1 <<", " << goal_pose.theta <<std::endl;
}

void AdsPlanner::setStaticMap(boost::shared_ptr<nav_msgs::OccupancyGrid> static_grid){
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
                        this->static_blocked.insert(mp(a,b));
            }
}

void AdsPlanner::setObstacleList(boost::shared_ptr<path_planner::PoseVelArray> obs_list){
    std::cout << "Detect Obstacles: " << obs_list->data.size() << std::endl;
    for(int i=0; i<obs_list->data.size(); i++){
        std::cout << "obstacle " << i 
                  << ", x=" << obs_list->data[i].pose.position.x 
                  << ", y=" << obs_list->data[i].pose.position.y 
                  << ", yaw=" << tf::getYaw(obs_list->data[i].pose.orientation)
                  << ", vel=" << obs_list->data[i].vel << std::endl;
    }
}

void AdsPlanner::setObstacleMap(boost::shared_ptr<nav_msgs::OccupancyGrid> obs_grid){
    int width = obs_grid->info.width;
    int height = obs_grid->info.height;
    this->blocked = this->static_blocked;
    this->is_obstacle_present = false;
    for(int yi=0; yi<width; yi++)
        for(int xi=0; xi<height; xi++)
            if(obs_grid->data[xi+yi*width] > 0)
            {
                //just replan if there is a obstacle
                this->is_obstacle_present = true;
                for(int a=max(0,xi-4);a<min(height,xi+5);a++)
                    for(int b=max(0,yi-4);b<min(width,yi+5);b++)
                        this->blocked.insert(mp(a,b));
            }
}



pdd AdsPlanner::key(Pose2D state){
	pii state2d = getpii(state);
        if(this->blocked.find(state2d)!=this->blocked.end()){
                cout<<"key for blocked state accessed"<<endl;
                return mp(0.1*DBL_MAX,0.1*DBL_MAX);
        }
        Pose2D initial_position = this->initial_position;
        double h_cost = max(abs(initial_position.x-state.x),abs(initial_position.y-state.y));
       // pii state2d = getpii(state);
        if(this->cost_to_goal[state2d]>this->rhs[state2d])
                return mp(this->rhs[state2d] + this->epsilon * h_cost,this->rhs[state2d]);
        else 
                return mp(this->cost_to_goal[state2d] +  h_cost,this->cost_to_goal[state2d]);
}

void  AdsPlanner::updateState(Pose2D state){
        set<pii> blocked = this->blocked;
        pii state2d = getpii(state);
        pii xrange = this->xrange, yrange = this->yrange;
        if(this->cost_to_goal.find(state2d)!=this->cost_to_goal.end())
                this->cost_to_goal[state2d] = 0.1*DBL_MAX;
        if(state.x != this->terminus.x || state.y != this->terminus.y){
                vector<Pose2D> successors = searchNextPoses(state, 5, 2, 1.5);
                double minh = 0.1*DBL_MAX;
                for(vector<Pose2D>::iterator it = successors.begin();it!=successors.end();it++){
                        pii nxt = getpii(*it);//cout<<"processing succ " ;disp2d(state);disp2d(*it);
                        double dx = abs(state.x-(*it).x), dy = abs(state.y-(*it).y);
                        double cost = sqrt(dx*dx+dy*dy);
                        if (blocked.find(nxt) != blocked.end() || nxt.first < xrange.first || nxt.first > xrange.second || nxt.second < yrange.first || nxt.second > yrange.second)continue;

                        //this->dads[nxt].pb(state);
                        if(minh > cost_to_goal[nxt] + cost)
                                minh = cost_to_goal[nxt] + cost;
                }
                this->rhs[state2d] = minh;
              //  if(minh>=0.1*DBL_MAX) cout<<"blocked state inserted"<<endl;
                }
        if(this->open.erase(state2d)){
        //this->open_st.erase(mp(state.x,state.y));
        priority_queue<pair<pii ,pdd>, vector<pair<pii,pdd> >, adscomp > tmp;
        while(!this->open_mp.empty()){
                pair<pii, pdd> tmpnode = this->open_mp.top();
                this->open_mp.pop();
                pii cnode = tmpnode.first;
                if(state.x!=cnode.first || state.y!=cnode.second) tmp.push(tmpnode);
        }
        this->open_mp=tmp;
        }
        if(blocked.find(state2d)!=blocked.end()){
                cost_to_goal[state2d] = 0.1 * DBL_MAX;
                rhs[state2d] = 0.1 * DBL_MAX;
        }               
        if(cost_to_goal[state2d]!=rhs[state2d]){
                if(closed.find(state2d)==closed.end()){
                        this->open_mp.push(mp(state2d,key(state)));
                        this->open.insert(state2d);
                        this->open_st[state2d] = state;
                }else{
                        this->incons.insert(state2d);
                        this->incons_st[state2d] = state;
                //      incons_mp.push(mp(state,key(state)));
                }
        }
}


Pose2D AdsPlanner::makep2d(pii a){
 Pose2D tmp;
        tmp.x =GRID_RESOLUTION* a.first;
        tmp.y =GRID_RESOLUTION* a.second;
        tmp.theta = 0;
return tmp;
}

pii AdsPlanner::getpii(Pose2D a){
	return mp(int(a.x/GRID_RESOLUTION),int(a.y/GRID_RESOLUTION));
}

vector<Pose2D>  AdsPlanner::computeOrImprovePath(){
    vector<Pose2D> path;
    pii iposn = getpii(this->initial_position);//cout<<"a";
    while(ros::ok() && !this->open.empty() &&
        (lexcomp(this->open_mp.top(),mp(iposn,key(this->initial_position))) 
            || this->rhs[iposn]!=this->cost_to_goal[iposn]) )
    {
        pair<pii, pdd> sx = this->open_mp.top();
        pii s = sx.first;//cout<<"b";       
        path.pb(this->open_st[s]);
        this->open_mp.pop();
        this->open.erase(s);
        vector<Pose2D> dads = searchNextPoses(this->open_st[s], 5, -2, 1.5);
        if(this->cost_to_goal[s] > this->rhs[s]){
            this->cost_to_goal[s] = this->rhs[s];
            this->closed.insert(s);//cout<<"c";
           /* for(vector<Pose2D>::iterator it=this->dads[s].begin();it!=this->dads[s].end();it++)
                    updateState(*it);
            }else{
            this->cost_to_goal[s] = 0.1 * DBL_MAX;
            for(vector<Pose2D>::iterator it=this->dads[s].begin();it!=this->dads[s].end();it++)*/
            for(vector<Pose2D>::iterator it=dads.begin();it!=dads.end();it++)
                updateState(*it);
        }else{
            this->cost_to_goal[s] = 0.1 * DBL_MAX;
            for(vector<Pose2D>::iterator it=dads.begin();it!=dads.end();it++)
                updateState(*it);
            updateState(this->open_st[s]);
        }
      //  cout<<"d";
        //this->open_st.erase(s);
    }
    //cout<<"e"<<endl;       
    return path;
}

nav_msgs::Path pose2d_to_path(vector<Pose2D> v){
    nav_msgs::Path path;
    path.header.frame_id = "/map";

    cout << "current path: [";
    for(vector<Pose2D>::iterator it=v.begin();it!=v.end();it++){
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "/map";
        ps.pose.position.x = (*it).x;
        ps.pose.position.y = (*it).y;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(it->theta);
        path.poses.push_back(ps);
        cout << "("<< it->x <<", " << it->y <<", "<< it->theta <<") ";
    }
    cout << "]"<< endl;

    return path;
}

void AdsPlanner::disp2d(Pose2D a){
	cout<< a.x<< " "<<a.y<< " "<<a.theta<< " "<<endl;
}

void AdsPlanner::start_planning_node(boost::shared_ptr<ros::NodeHandle> nh){
    ros::Publisher current_pose_pub = nh->advertise<geometry_msgs::PoseStamped>("current_pose",1);
    ros::Publisher path_pub = nh->advertise<nav_msgs::Path>("global_path",1);
    ros::Subscriber obs_sub = nh->subscribe("obstacle_cost_map", 1, &AdsPlanner::setObstacleMap, this);
    
    Pose2D initial_position = this->initial_position;
    Pose2D terminus = this->terminus;
    pii iposn = getpii(initial_position), tposn = getpii(terminus);
    this->cost_to_goal[iposn] = 0.1 * DBL_MAX;
    this->rhs[iposn] = 0.1 *  DBL_MAX;
    this->cost_to_goal[tposn] = 0.1 *  DBL_MAX;
    this->rhs[tposn] = 0;
    this->epsilon = 2.5;
    this->open_mp = priority_queue<pair<pii ,pdd>, vector<pair<pii,pdd> >, adscomp >();
    //      this->closed_mp = priority_queue<pair<pii, pdd> >();
    //      this->incons_mp = priority_queue<pair<pii, pdd> >();
    this->open = set<pii>();
    this->closed = set<pii>();
    this->incons = set<pii>();
    this->open_st = map<pii, Pose2D>();
    this->incons_st = map<pii, Pose2D>();

    this->open_mp.push(mp(tposn, key(terminus)));
    this->open.insert(tposn);
    this->open_st[tposn] = terminus;

    vector<Pose2D> path = computeOrImprovePath();
    cout << "Path len: " << path.size() << endl;
    path_pub.publish(pose2d_to_path(path));
    

    bool change1=0;

    while(ros::ok() & (terminus.x!=initial_position.x || terminus.y!=initial_position.y)){
        set<pii> changed_locs;

        //publish

        //update inital position
        this->initial_position = path.back();
        path.pop_back();
        initial_position = this->initial_position;

        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time::now();
        ps.pose.position.x = initial_position.x;
        ps.pose.position.y = initial_position.y;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(initial_position.theta);
        current_pose_pub.publish(ps);

        set<pii> obsmap = this->blocked;
        
        bool change2 = this->is_obstacle_present;

        if(change1 || change2 || epsilon>1){
            if((!change1) && change2)
                this->epsilon = 2.5;

            if(change1||change2)
            {               
                for(set<pii>::iterator it=obsmap.begin();it!=obsmap.end();it++)
                    if(this->blocked.find(*it)==this->blocked.end())
                        changed_locs.insert(*it);
                for(set<pii>::iterator it=this->blocked.begin();it!=this->blocked.end();it++)
                    if(obsmap.find(*it)==obsmap.end())
                    changed_locs.insert(*it);
            }

            change1 = change2;

            for(set<pii>::iterator it=changed_locs.begin();it!=changed_locs.end();it++)
                updateState(makep2d(*it));

            if(this->epsilon>1) this->epsilon-=0.1;
    
            priority_queue<pair<pii ,pdd>, vector<pair<pii,pdd> >, adscomp > tmp;
            //while(!this->incons.empty()){
            for(set<pii>::iterator it = this->incons.begin(); it!=this->incons.end(); it++){
                this->open.insert(*it);
                this->open_st[*it]=this->incons_st[*it];                                
            }
            this->incons.clear();
            for(set<pii>::iterator it = this->open.begin(); it!=this->open.end(); it++){
                tmp.push(mp(*it,key(open_st[*it])));
            }
            this->open_mp = tmp;
            this->closed = set<pii>();

            path =  computeOrImprovePath();
        }
    }
}
#endif
