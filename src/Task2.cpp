
//MOTION CONTROL//

/*Modelled on the following Research Paper:
//”Control of Wheeled Mobile Robots: An Experimental Overview” by Alessandro De Luca, Giuseppe Oriolo, Marilena Vendittelli,
//Dipartimento di Informatica e Sistemistica, Universita degli Studi di Roma “La Sapienza”, Italy
*/

#include <iostream>
#include "ros/ros.h"
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>

#include "swarm_simulator/obstacleList.h"
#include "swarm_simulator/obstacleData.h"

using namespace std;

typedef typename geometry_msgs::Twist Twist;

#define curr_x (getmodelstate.response.pose.position.x-fin_x)
#define curr_theta (normalizeAngle(2*acos(getmodelstate.response.pose.orientation.w)-fin_theta))
#define curr_y (getmodelstate.response.pose.position.y-fin_y)
#define threshold 0.1

double normalizeAngle(double angle) 
{
	return angle - (2*M_PI*floor((angle + M_PI)*(M_1_PI)/2));  
}

//double theta(Pose pos) 
//{ //returns theta of pos wrt x-axis
//  return normalizeAngle(2 * atan2(pos.orientation.z, pos.orientation.w));
//}

class path_parameters//REQUIRED FOR DEFINING A PATH(set of 3 for a 2D plane)
{
	public:
	double k1;
	double k2;
	double k3;
	void set(double x1, double x2, double x3)
	{
		k1=x1;
		k2=x2;
		k3=x3;
	}
};

//Fuction to convert cartesian to polar. Applied verbatim from the Research Paper on which this solution has been modelled
class Polar
{
	public:
	double ro;
	double gamma;
	double delta;
	Polar(double x, double y, double theta)
	{
		ro=sqrt(x*x + y*y);
		gamma=normalizeAngle(atan2(y,x)-theta+M_PI);			
		delta=normalizeAngle(gamma+theta);
	}
	Polar();
	void set(double x, double y, double theta)
	{
		ro=sqrt(x*x + y*y);
		gamma=normalizeAngle(atan2(y,x)-theta+M_PI);			
		delta=normalizeAngle(gamma+theta);
	}
	double calc_v(path_parameters p)
	{
		double v;
		v=p.k1*ro*cos(gamma);
		return v;
	}
	double calc_w(path_parameters p)
	{
		double w;
		if(gamma==0)
			w=p.k1*p.k3*delta;
		else
			w=p.k2*gamma+p.k1*sin(gamma)*cos(gamma)*(gamma+p.k3*delta)/gamma;
		return w;
	}
};


int main(int argc, char **argv)
{	
	double fin_x;
	double fin_y;
	double fin_theta;
	string name;
	path_parameters p;
	ros::init(argc, argv, "Motion_Control");

	//Entering Requisite Terminal Parameters
	
  	ros::NodeHandle terminal_handle("~");
	terminal_handle.param("name", name, std::string("swarmbot0"));
  	terminal_handle.param("final_x", fin_x, double(0));//Keeping default final destinations 
  	terminal_handle.param("final_y", fin_y, double(0));//to 0, 0, 0 by default ( x, y, theta )
	terminal_handle.param("final_theta", fin_theta, double(0));

	//NodeHandle, Subscribers, Service Client declared; gazebo messages for model states declared
  	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	gazebo_msgs::GetWorldProperties prop;
	ros::Publisher vel_pub_0 = n.advertise<Twist>("/swarmbot0/cmd_vel", 1);
	ros::ServiceClient service_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState getmodelstate;
	gazebo_msgs::ModelState modelstate;
	ros::Rate loop_rate(50);// Loop rate defined

	int count = 0;//Just to keep track of the number of iterations of message transferred
	Twist cmd_vel;
	getmodelstate.request.model_name = name;
	service_client.call(getmodelstate); 
	Polar polar_cord(curr_x,curr_y,curr_theta);//Current coordinates in polar form
	p.set(0.75,4,20);//Set path parameters for the start
	ROS_INFO("%lf\n",polar_cord.ro);

	while((ros::ok()) && (polar_cord.ro > threshold))
 	{
		service_client.call(getmodelstate); 
		polar_cord.set(curr_x,curr_y,curr_theta);
		
		cmd_vel.linear.x = polar_cord.calc_v(p);
		cmd_vel.angular.z = polar_cord.calc_w(p);

		ROS_INFO("%lf %lf %lf %lf %lf",curr_x, curr_y, curr_theta, cmd_vel.linear.x, cmd_vel.angular.z);
		ROS_INFO("%lf %lf %lf",polar_cord.ro, polar_cord.gamma, polar_cord.delta);
    	vel_pub_0.publish(cmd_vel);
    	ros::spinOnce();
   		loop_rate.sleep();
   		++count;
	}
	
	//Stopping the Bot
	cmd_vel.linear.x=0;
	cmd_vel.angular.z=0;
	vel_pub_0.publish(cmd_vel);
    ros::spinOnce();
  	return 0;
}
