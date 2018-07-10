#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "gazebo_msgs/GetModelState.h"
#include <gazebo_msgs/SetModelState.h>

#include <string>
#include <fstream>
#include <iostream>
	
int main(int argc, char **argv)
{
	
	float a[2200][3];
	std::ifstream f("/home/shizu/myws/src/uav/waypoints.txt", std::ios::in);
	for(int j=1;j<10;j++)
	f>>a[j][1]>>a[j][2]>>a[j][3];
	f.close();
	
	ros::init(argc, argv, "vive_controller");
	ros::NodeHandle nh;
	ros::Rate r(90);
	
	
	ros::Publisher base_control;
	base_control = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	geometry_msgs::Twist base_motion;
	
	
	system("rosservice call /enable_motors true");
	
	ros::ServiceClient client_get = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_state;
	get_state.request.model_name = "quadrotor";

	ros::ServiceClient client_set = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	gazebo_msgs::SetModelState set_state;
	set_state.request.model_state.model_name = "quadrotor";
	
	std::cout << set_state.request << std::endl;
	
	set_state.request.model_state.pose.position.x = 0;
	set_state.request.model_state.pose.position.y = 0;
	set_state.request.model_state.pose.position.z = 0.2;
	set_state.request.model_state.pose.orientation.x = 0;
	set_state.request.model_state.pose.orientation.y = 0;
	set_state.request.model_state.pose.orientation.z = 0;
	set_state.request.model_state.pose.orientation.w = 1;
	set_state.request.model_state.twist.linear.x = 0;
	set_state.request.model_state.twist.linear.y = 0;
	set_state.request.model_state.twist.linear.z = 0;
	set_state.request.model_state.twist.angular.x = 0;
	set_state.request.model_state.twist.angular.y = 0;
	set_state.request.model_state.twist.angular.z = 0;
	
	client_set.call(set_state);
	
	int i = 0;
	
	float x_tar;
	float y_tar;
	float z_tar;
	
	float kp_x = 1;
	float kd_x = 0.3;
	
	float kp_z = 2;
	float kd_z = 0.4;
	
	while(ros::ok())
	{
		
		x_tar = a[i][1];
		y_tar = a[i][2];
		z_tar = a[i][3];
		
		client_get.call(get_state);
		std::cout << x_tar <<" , "<< y_tar <<" , " << z_tar << std::endl;
		std::cout <<"x = "<< get_state.response.pose.position.x << std::endl;
		std::cout <<"y = "<< get_state.response.pose.position.y << std::endl;
		std::cout <<"z = "<< get_state.response.pose.position.z << std::endl<< std::endl;
		
		

		base_motion.linear.x = (x_tar - get_state.response.pose.position.x)*kp_x - get_state.response.twist.linear.x * kd_x;
		base_motion.linear.y = (y_tar - get_state.response.pose.position.y)*kp_x - get_state.response.twist.linear.y * kd_x;
		
		base_motion.linear.z = (z_tar - get_state.response.pose.position.z)*kp_z - get_state.response.twist.linear.z * kd_z;

		base_control.publish(base_motion);
		
		float diff = pow((x_tar - get_state.response.pose.position.x),2) + pow((y_tar - get_state.response.pose.position.y),2)
					 + pow((y_tar - get_state.response.pose.position.y),2);
					 
		if (diff<1)
		{
				i ++;
		}
		
		if (i==10){i=1;}
		
	}

}
