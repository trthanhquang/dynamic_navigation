#ifndef DIJKSTRA_PLANNER
#define DIJKSTRA_PLANNER

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include <cstdlib>
#include <queue>
#include <stdio.h>
#include <limits.h>
// #include <map>
#include <vector>
#include <set>
#include <utility>

#define mp make_pair
#define pb push_back

using namespace std;
typedef pair<int, int> pii;
typedef pair<int, pii> pi3;

class DijkstraPlanner{
public:
    DijkstraPlanner(){
    }
    void setInit(double x, double y, double thetha);
    void setGoal(double x, double y, double thetha);
    void setMap(nav_msgs::OccupancyGrid occup_grid);
    nav_msgs::Path solve_path(void);
private:
    set<pii> blocked;
};

void DijkstraPlanner::setMap(nav_msgs::OccupancyGrid occup_grid){
    int width = occup_grid.info.width;
    int height = occup_grid.info.height;
    //clear block
    for(int i=0; i<width; i++)
        for(int j=0; j<height; j++)
            if(occup_grid.data[i*width+height] > 0)
                this->blocked.insert(std::make_pair(i,j));
}

#endif