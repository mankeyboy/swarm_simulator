
//A* Path Implementation

#include <iostream>
#include "ros/ros.h"
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>
#include <queue>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"

using namespace std;

typedef typename geometry_msgs::Twist Twist;

std::vector<swarm_simulator::obstacleData> obstacles(3);
const int X_MAX = 100;
const int Y_MAX = 100;
const int GRID_X1 = -12;
const int GRID_X2 = 12;
const int GRID_Y1 = -12;
const int GRID_Y2 = 12;

//Basic structure of the graph node
typedef struct point
{
    double x;
    double y;
}point;

//DEFINING OPERATORS FOR POINT STRUCTURE
bool operator==(const point& lhs, const point& rhs)
    {
        return ((lhs.x == rhs.x) && (lhs.y == rhs.y));
    }
bool operator!=(const point& lhs, const point& rhs)
    {
        return ((lhs.x != rhs.x) || (lhs.y != rhs.y));
    }

//FUNCTION TO CALCULATE DISTANCE BETWEEN a AND b
double dist(const point& a, const point& b)
{
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

//CHECK IF POINT P LIES ON AN OBSTACLE OR IF IT IS OUTSIDE THE GRID
int check(point p)
{
    point temp;
    if(p.x<0 || p.x > X_MAX)
        return 0;
    if(p.y<0 || p.y > Y_MAX)
        return 0;
    for(std::vector<swarm_simulator::obstacleData>::iterator i=obstacles.begin(); i!=obstacles.end(); i++)
    {
        temp = {(*i).x,(*i).y};
        if(dist(temp,p) < (*i).radius)
            return 0;
    }
    return 1;
}

class Que//CLASS FOR MEMBERS OF THE QUEUE FOR A*
{
    public:
    point p;
    double exp_dist;//estimated total distance
};

struct compare//COMPARISON OPERATOR FOR THE PRIORITY QUEUE
 {
   bool operator()(const Que& l, const Que& r)
   {
       return l.exp_dist > r.exp_dist;
   }
 };

std::list<point> create_A_Star_Path(point start, point fin)
{
    int i,j;
    double temp1,temp2;
    Que curr, tempq;
    point temp_point;
    std::priority_queue< Que ,std::vector<Que> , compare> q;
    std::vector<std::vector<point> > parent (X_MAX + 1);//To implement the parent-child relation from initial to final point
    std::vector<std::vector<double> > distances(X_MAX+1);//The distances of those points
    std::list<point> sequence;//Final sequence
    std::vector<std::vector<int> > visited (X_MAX + 1);

    for(i=0; i<=X_MAX; i++)
    {
        visited[i] = std::vector<int> (Y_MAX + 1);
        distances[i]=std::vector<double> (Y_MAX+1);
        parent[i]=std::vector<point> (Y_MAX+1);
        for(j=0; j<=Y_MAX;j++)
        {
            visited[i][j]=0;
            distances[i][j]=0;
        }
    }
    curr.p = start;
    curr.exp_dist = 0;
    distances[start.x][start.y]=0;
    q.push(curr);
//EXACT A-STAR ALGORITHM.
    while(!q.empty())
    {
        curr = q.top();
        q.pop();
        visited[curr.p.x][curr.p.y]=1;
        if(curr.p==fin)
        {
            break;
        }
        for(i=curr.p.x-1; i<=curr.p.x+1; i++)
        {
            for(j=curr.p.y-1; j<=curr.p.y+1; j++)
            {
                temp_point = {i,j};
                if(temp_point == curr.p)
                    continue;
                if(check(temp_point)==1)
                {

                    temp2 = distances[curr.p.x][curr.p.y] + dist(temp_point, curr.p);
                    temp1 = dist(temp_point,fin) + temp2;
                    
                    if(visited[i][j]==1 && temp2 >= distances[i][j])
                        continue;
                    parent[i][j]=curr.p;
                    tempq.p = temp_point;
                    tempq.exp_dist = temp1;
                    distances[i][j] = temp2;
                    q.push(tempq);
                }
            }
        }
    }
    temp_point = fin;
    while(temp_point!=start)
    {
        sequence.push_front(temp_point);
        temp_point=parent[temp_point.x][temp_point.y];
    }
    return sequence;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "A_Star_Implementation");
  	//std::vector<swarm_simulator::obstacleData > x(3);
  	std::list<point> sequence;
	//SAMPLE OBSTACLE LIST
  	point start={20,20},fin={90,90};
  	obstacles[0].x=30;
  	obstacles[0].y=30;
  	obstacles[0].radius=10;
  	obstacles[1].x=45;
  	obstacles[1].y=15;
  	obstacles[1].radius=2;
  	obstacles[2].x=15;
  	obstacles[2].y=45;
  	obstacles[2].radius=2;
  	sequence = create_A_Star_Path(start,fin);
  	for(std::list<point>::iterator i = sequence.begin(); i!=sequence.end(); i++)
  	{
        cout<<(*i).x<<" "<<(*i).y<<endl;
  	}
  	return 0;
}
