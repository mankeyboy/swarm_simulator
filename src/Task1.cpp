
// 8 - Like Crossed Looping Motion

#include "ros/ros.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <utility>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>

using namespace std;
typedef typename geometry_msgs::Twist Twist;

const int desx = 0;
const int desy = 0;
const int NUM_LOOPS = 2;//This number is the number of times the bot will complete one looping motion
#define curr_x getmodelstate.response.pose.position.x-desx
#define curr_y getmodelstate.response.pose.position.y-desy

int main(int argc, char **argv)
{
	string name;
	double v,w;
  	ros::init(argc, argv, "Task_1_8_Looping_Motion");

	//TO ACCEPT PARAMETERS FROM YOUR TERMINAL
	ros::NodeHandle terminal_handle("~");
	terminal_handle.param("name", name, std::string("swarmbot0"));
  	terminal_handle.param("v", v, double(10));//Keeping v & w equal 
  	terminal_handle.param("w", w, double(10));//to 10 by default

	//DECLARING CLIENT AND PUBLISHER
  	ros::NodeHandle n;
   	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  	gazebo_msgs::GetWorldProperties prop;
  	ros::Publisher vel_pub_0 = n.advertise<Twist>("/swarmbot0/cmd_vel", 1);
	std::string s = name;
  	ros::ServiceClient service_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  	gazebo_msgs::GetModelState getmodelstate;
  	gazebo_msgs::ModelState modelstate;
  	geometry_msgs::Twist cmd_vel;

  	//LOOP RATE for simulation (Not to be confused with the Loop Motion to be simulated)
  	ros::Rate loop_rate(50);

  	int count = 0;
	getmodelstate.request.model_name = s;//ENABLES service_client.call() TO FETCH THE CORRECT MODEL DATA.

	//LOOP
	for( int i = 0; i < NUM_LOOPS; i++ )
	{
		cmd_vel.linear.x = v;
		cmd_vel.linear.y = 0;
		cmd_vel.linear.z = 0;
    	
    	cmd_vel.angular.x = 0;
    	cmd_vel.angular.y = 0;
    	cmd_vel.angular.z = pow(-1,i)*w*(-1);

		vel_pub_0.publish(cmd_vel);
		service_client.call(getmodelstate);
		ROS_INFO("Loop %d", i);
		while(ros::ok())
 		{
			if((i%2 == 0 && getmodelstate.response.pose.orientation.w < -0.99) || (i%2 == 1 && getmodelstate.response.pose.orientation.w > 0.99))//CHECK IF LOOP IS COMPLETE (Blame double for the 0.99)
			{
				break;
			}
			service_client.call(getmodelstate);    
 			ROS_INFO("%lf %lf %lf",curr_x,curr_y,getmodelstate.response.pose.orientation.w);

			vel_pub_0.publish(cmd_vel);
    		ros::spinOnce();
    		loop_rate.sleep();
    		++count;
  		}
  	}

  	//STOP THE BOT AFTER LOOPING MOTION IS OVER
  	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
	vel_pub_0.publish(cmd_vel);
  	return 0;	
}