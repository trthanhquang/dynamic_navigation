/* 
 * File:   main.cpp
 * Author: hackd
 *
 * Created on November 7, 2015, 11:00 PM
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <queue>
#include <stdio.h>
#include <limits.h>
#include <map>
#include <vector>
#include <set>
#include <utility>

#define mp make_pair
#define pb push_back

using namespace std;

typedef pair<int, int> pii;
typedef pair<int, pii> pi3;

const int INF = INT_MAX;
 set<pii> blocked;

void parsemap(const std_msgs::String::ConstPtr& msg){
   int mapdata[][] = msg->data;
   for(int i=0;i<10;i++)
     for(int j=0;j<10;j++)
	if(mapdata[i][j]>0)
	blocked.insert(mp((i,j)));
}


int main() {
    //declarations
    priority_queue<pi3, vector<pi3>, greater<pi3> > Q;
    map<pii, int> dist;
    map<pii, pii> dad;

    pii xrange, yrange, initial_position = mp(2, 2), t;
//    set<pii> blocked;

    //dummy
    xrange = mp(1, 10);
    yrange = mp(1, 10);
    t = mp(6, 5);

    ros::init(argc, argv "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("map", 1000, parsemap);
/*    blocked.insert(mp(3, 3));
    blocked.insert(mp(4, 4));
    blocked.insert(mp(5, 5));
    blocked.insert(mp(8, 8));
    blocked.insert(mp(7, 6));
    blocked.insert(mp(6, 7));
*/

    //initializations
    Q.push(mp(0, initial_position));
    dist[initial_position] = 0;

    //Djikstra
    while (!Q.empty()) {
        pi3 p = Q.top();
        if ((p.second).first == t.first && (p.second).second == t.second) break;
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


    if (dist.find(t) != dist.end())
        for (pii i = t; dad.find(i) != dad.end(); i = dad[i])
            printf("%d %d%c", i.first, i.second, ((i.first == initial_position.first && i.second == initial_position.second) ? '\0' : '\n'));

    return 0;
}
