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
#include "boost/shared_ptr.h"


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

/*struct Pose2D{
    double x;
    double y;
    double theta;
};*/

class adscomp{
	public:
	bool operator() (const pair<pii ,pdd>& a,const pair<pii ,pdd>& b) const{
		if(a.second.first == b.second.first) return a.second.second < b.second.second;
		else return a.second.first < b.second.first;
	}
}

class AdsPlanner{
public:
    AdsPlanner(){
    }
    void setCurrentTime(ros::Time now);
    void setCurrentPose(Pose2D current_pose);
    void setGoalPose(Pose2D goal_pose);
    void setStaticMap(boost::shared_ptr<nav_msgs::OccupancyGrid> occup_grid);
    int setObstacleMap(boost::shared_ptr<nav_msgs::OccupancyGrid> occup_grid);
    void setObstacleList(boost::shared_ptr<path_planner::PoseVelArray> obs_list);
    nav_msgs::Path findPath(void);
    
    vector<Pose2D> searchNextPoses(Pose2D pose, double output_size, double vel, double dt);

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
    map<pii, vector<Pose2D> > dads;
    map<pii, Pose2D> open_st,incons_st;
};

bool lexcomp ( pair<pii ,pdd> a,pair<pii ,pdd> b){
                if(a.second.first == b.second.first) return a.second.second < b.second.second;
                else return a.second.first < b.second.first;
}


vector<Pose2D> AdsPlanner::searchNextPoses(Pose2D cur_pose, double output_size, double vel, double dt){
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
                        this->static_blocked.insert(std::make_pair(a,b));
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

int AdsPlanner::setObstacleMap(boost::shared_ptr<nav_msgs::OccupancyGrid> obs_grid){
	//std::cout << "Detect Obstacles: " << obs_list->data.size() << std::endl;
	this->blocked = this->static_blocked;
	double obs_rad = this->obs_rad;
	
    for(int i=0; i<obs_list->data.size(); i++){
	double xi =  data[i].pose.position.x, yi = data[i].pose.position.y;
                  for(int a=max(0,xi-obs_rad);a<min(yrange.second,xi+obs_rad);a++)
                     for(int b=max(0,yi-obs_rad);b<min(xrange.second,yi+obs_rad);b++)
                        if(sqrt(a*a+b*b)<=obs_rad)
			this->blocked.insert(mp(a,b));
    }
    return obs_list->data.size();
}

pdd AdsPlanner::key(Pose2D state){
	if(this->blocked.find(mp(state.x,state.y))!=this->blocked.end()){
		cout<<"key for blocked state accessed"<<endl;
		return mp(0.1*DBL_MAX,0.1*DBL_MAX);
	}
	Pose2D initial_position = this->initial_position;
	double h_cost = max(abs(initial_position.x-state.x),abs(initial_position.y-state.y));
	pii state2d = mp(state.x,state.y);
	if(cost_to_goal[state2d]>rhs[state2d])
		return mp(this->rhs[state2d] + this->epsilon * h_cost,rhs[state2d]);
	else 
		return mp(this->cost_to_goal[state2d] +  h_cost,cost_to_goal[state2d]);
}

void  AdsPlanner::updateState(Pose2D state){
	set<pii> blocked = this->blocked;
	pii xrange = this->xrange, yrange = this->yrange;
	if(cost_to_goal.find(mp(state.x,state.y))!=cost_to_goal.end())
		cost_to_goal[mp(state.x,state.y)] = 0.1*DBL_MAX;
	if(state.x != this->terminus.x || state.y != this->terminus.y){
		vector<Pose2d> successors = searchNextPoses(state, 5, 2, 1.5);
		double minh = 0.1*DBL_MAX;
		for(vector<Pose2D>::iterator it = successors.begin();it!=successors.end();it++){
			pii nxt = mp((*it).x, (*it).y);
			double dx = abs(state.x-(*it).x), dy = abs(state.y-(*it).y);
			double cost = sqrt(dx*dx+dy*dy);
			if (blocked.find(nxt) != blocked.end() || nxt.first < xrange.first || nxt.first > xrange.second || nxt.second < yrange.first || nxt.second > yrange.second)continue;

			this->dads[nxt].pb(state);
			if(minh > cost_to_goal[nxt] + cost)
				minh = cost_to_goal[nxt] + cost;
		}
		this->rhs[mp(state.x,state.y)] = minh;
		if(minh>=0.1*DBL_MAX) cout<<"blocked state inserted"<<endl;
		}
	if(this->open.erase(mp(state.x,state.y))){
	//this->open_st.erase(mp(state.x,state.y));
	priority_queue<pair<pii,pdd> > tmp;
	while(!this->open_mp.empty()){
		pair<pii, pdd> tmpnode = this->open.top();
		this->open_mp.pop();
		pii cnode = tmpnode.first;
		if(state.x!=cnode.first || state.y!=cnode.second) tmp.push[tmpnode];
	}
	this->open_mp=tmp;
	}
	if(blocked.find(mp(state.x,state.y))!=blocked.end()){
		cost_to_goal[mp(state.x,state.y)] = 0.1 * DBL_MAX;
		rhs[mp(state.x,state.y)] = 0.1 * DBL_MAX;
}		
	if(cost_to_goal[mp(state.x,state.y)]!=rhs[mp(state.x,state.y)]){
		if(closed.find(mp(state.x,state.y))==closed.end()){
			this->open_mp.push(mp(mp(state.x,state.y),key(state)));
			this->open.insert(mp(state.x,state.y));
			this->open_st[mp(state.x,state.y)] = state;
		}else{
			this->incons.insert(mp(state.x,state.y));
			this->incons_st[mp(state.x,state.y)] = state;
		//	incons_mp.push(mp(state,key(state)));
		}
	}
}


Pose2D AdsPlanner::makep2d(pii a){
 Pose2D tmp;
	tmp.x = a.first;
	tmp.y = a.second;
	tmp.theta = 0;
return tmp;
}

vector<Pose2D>  AdsPlanner::computeOrImprovePath(){
	vector<Pose2D> path;
	pii iposn = mp(this->initial+position.x,this->initial_position.y);
	while(lexcomp(this->open_mp.top(),mp(iposn,key(this->initial_position))) || this->rhs[iposn]!=this->cost_to_goal[iposn]){
	pair<pii, pdd> sx = this->open_mp.top();
	pii s = sx.first;	
	path.pb(this->open_st[s]);
	this->open_mp.pop();
	this->open.erase(s);
	if(this->cost_to_goal[s] > this->rhs[s]){
		this->cost_to_goal[s] = this->rhs[s];
		this->closed.insert(s);
		for(vector<Pose2D>::iterator it=this->dads[s].begin();it!=this->dads[s].end();it++)
			updateState(*it);
		}else{
		this->cost_to_goal[s] = 0.1 * DBL_MAX;
		for(vector<pii>::iterator it=dads[s].begin();it!=dads[s].end();it++)
                        updateState(*it);
		updateState(open_st[s]);
		}
	//this->open_st.erase(s);
	}	
	return path;
}


nav_msgs::Path AdsPlanner::findPath(){
	intial_state = this->initial_state;
	terminus = this->terminus;
	this->cost_to_goal[initial_state] = 0.1 * DBL_MAX;
	this->rhs[initial_state] = 0.1 *  DBL_MAX;
	this->cost_to_goal[terminus] = 0.1 *  DBL_MAX;
	this->rhs[terminus] = 0;
	this->epsilon = 2.5;
	this->open_mp = priority_queue<pair<pii, pdd> >();
//	this->closed_mp = priority_queue<pair<pii, pdd> >();
//	this->incons_mp = priority_queue<pair<pii, pdd> >();
	this->open = set<pii>();
	this->closed = set<pii>();
	this->incons = set<pii>();
	this->open_st = map<pii, Pose2D>();
	this->incons_st = map<pii, Pose2D>();
	
	this->open_mp.push(mp(terminus, key(terminus)));
	this->open.insert(mp(terminus.x,terminus.y));
	this->open_st[mp(terminus.x,terminus.y)] = terminus;
	
	vector<Pose2D> path = computeOrImprovePath();
	
	

//publish

//while sterm sgoal
bool change1=0;

	while(terminus.x!=initial_position.x || terminus.y!=initial_position.y){
		set<pii> changed_locs;

		//publish

		//update inital position
		this->initial_position = path.pop_back();
		intial_posiiton = this->initial_position;

		ros::NodeHandle n;
		ros::Publisher path_pub = n.advertise<geometry_msgs::Pose2D>("global_path",1000);
		path_pub.publish(this->initial_position);


		set<pii> obsmap = this->blocked;
		bool change2 = setObstacleMap()>0;
	if(change1 || change2 || epsilon>1){
	
	if((!change1) && change2) this->epsilon = 2.5;

	    if(change1||change2){		
		for(set<pii>::iterator it=obsmap.begin();it!=obsmap.end();it++)
		if(this->blocked.find(*it)==this->blocked.end())
		changed_locs.insert(*it);
		for(set<pii>::iterator it=this->blocked.begin();it!=this->blocked.end();it++)
                if(this->obsmap.find(*it)==this->obsmap.end())
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
                        tmp.pb(mp(*it,key(open_st[*it])));
                }
		this->open_mp = tmp;
		this->closed = set<pii>();

 		static_path =  computeOrImprovePath();
	}
	}
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
