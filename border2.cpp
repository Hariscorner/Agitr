#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include<tf/transform_broadcaster.h>

ros::Publisher *pubPtr;

void PoseReceived(const turtlesim::Pose& PoseIn){
	geometry_msgs::Twist msgOut;
	PoseIn.x;
	PoseIn.y;
	PoseIn.theta;
	PoseIn.linear_velocity;
	PoseIn.angular_velocity;
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "Pose=(" << PoseIn.x << "," << PoseIn.y << "," << PoseIn.theta << ")"
	<< "_vel=(" << PoseIn.linear_velocity << ","  << PoseIn.angular_velocity << ")" );
	
	float a=PoseIn.theta;

	if (PoseIn.x > 10.5)
	{	
		if ((a >= 0 && a < M_PI/2) || (a > -2*M_PI && a < -1.5*M_PI/2))
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = M_PI;	
		}
		else if ((a < 0 && a > -M_PI/2) || (a < 2*M_PI && a > 1.5*M_PI/2)) 
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = -M_PI;	
		}
		else
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = PoseIn.theta;	
		}
		pubPtr->publish(msgOut);
	}
	else if (PoseIn.x < 0.5)
	{	
		if ((a > 0.5*M_PI && a <= M_PI) || (a > -1.5*M_PI && a < -M_PI))
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = -M_PI;	
		}
		else if ((a < 1.5*M_PI && a > M_PI) || (a < -0.5*M_PI && a > -M_PI)) 
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = M_PI;	
		}
		else
		{
			msgOut.linear.x = 1;
			msgOut.angular.z = PoseIn.theta;	
		}
		pubPtr->publish(msgOut);
	}
	else if (PoseIn.y > 10.5)
	{
		msgOut.linear.x = 1;
		msgOut.angular.z = (PoseIn.theta > 1.5*M_PI ) ? 0.5*M_PI : 2.5*M_PI;
		pubPtr->publish(msgOut);	
	}
	else if (PoseIn.y < 0.5)
	{
		msgOut.linear.x = 1;
		msgOut.angular.z = (PoseIn.theta > 0.5*M_PI ) ? 1.5*M_PI: -0.5*M_PI;
		pubPtr->publish(msgOut);	
	}
	else
	{
		msgOut.linear.x = 	2;
		msgOut.angular.z = 	.05*M_PI;		
		//msgOut.linear.x = 	PoseIn.linear_velocity;
		//msgOut.angular.z = 	PoseIn.angular_velocity;
		ROS_INFO_STREAM("No wall in proximity");
		pubPtr->publish(msgOut);
	}

	
	//make changes

}

int main(int argc, char **argv){
	ros::init(argc, argv, "reverse_velocity");
	ros::NodeHandle nh;

	pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel_reversed",1000));
	
	ros::Subscriber sub = nh.subscribe("turtle1/pose",1000,&PoseReceived);

	ros::spin();

	delete pubPtr;
}
