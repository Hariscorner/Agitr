//Author: Hari Prasanth
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include<tf/transform_broadcaster.h>
#include <unistd.h>

ros::Publisher *pubPtr;

void PoseReceived(const turtlesim::Pose& PoseIn){
	geometry_msgs::Twist msgOut;
	//	PoseIn.x;
	//	PoseIn.y;
	//	PoseIn.theta;
	//	PoseIn.linear_velocity;
	//	PoseIn.angular_velocity;
	
	//Convert theta into principal values by first converting it into quarternion and then back to RPY. The Yaw angle hence obtained is principal value (-pi, pi)
	tf::Quaternion q;				//create a quarternion object
	q.setRPY(0,0,PoseIn.theta);		//initialize the quarternion with the theta value
	tf::Matrix3x3 m(q);				//create an RPY matrix and initialize it with the quarternion. This converts the quarternion into RPY principal values, but in matrix form
    double roll, pitch, yaw;		
    m.getRPY(roll, pitch, yaw);		//Extract individual values of RPY from the matrix
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "Pose=(" << PoseIn.x << "," << PoseIn.y << "," << PoseIn.theta << ")"
	<< "_vel=(" << PoseIn.linear_velocity << ","  << PoseIn.angular_velocity << ")" << ","  << q.getAngle() << "," << yaw);
	
	//float a=PoseIn.theta;

	if (PoseIn.x > 10.5)
	{	
		if (yaw >= 0 )
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = M_PI;	
		}
		else if (yaw < 0 )
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = -M_PI;	
		}
		else
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = yaw;	
		}
		pubPtr->publish(msgOut);
	}
	else if (PoseIn.x < 0.5)
	{	
		if (yaw <= M_PI && yaw > 0)
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = -M_PI;	
			
		}
		else if (yaw > -M_PI && yaw < 0)
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = M_PI;	
		}
		else
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = yaw;	
		}
		pubPtr->publish(msgOut);
		//usleep(1000000);
	}
	else if (PoseIn.y < 0.5 ) 	
	{
		if (yaw >= -0.5*M_PI && yaw <= 0.5*M_PI )
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = M_PI;	
		}
		else if (yaw < -0.5*M_PI || yaw <= M_PI)
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = -M_PI;	
		}
		else
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = yaw;	
		}
		pubPtr->publish(msgOut);
	}
	else if (PoseIn.y > 10.5)
	{
		if (yaw < 0.5*M_PI && yaw > -M_PI/2)
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = -M_PI;	
		}
		else if (yaw >= 0.5*M_PI || yaw >= -M_PI)
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = M_PI;	
		}
		else
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = yaw;	
		}
		pubPtr->publish(msgOut);
		//usleep(500000);
	}
	else
	{
		//msgOut.linear.x = 	2;
		//msgOut.angular.z = 	.05*M_PI;		
		//msgOut.linear.x = 	PoseIn.linear_velocity;
		//msgOut.angular.z = 	PoseIn.angular_velocity;
		ROS_INFO_STREAM("No wall in proximity");
		//pubPtr->publish(msgOut);
	}

	

}

int main(int argc, char **argv){
	ros::init(argc, argv, "reverse_velocity");
	ros::NodeHandle nh;

	pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel_reversed",1000));
	
	ros::Subscriber sub = nh.subscribe("turtle1/pose",1000,&PoseReceived);
	
ros::Rate my_rate(60);
	while(ros::ok() && nh.ok())
	{
		ros::spinOnce();
		my_rate.sleep();
	}

	delete pubPtr;
}
