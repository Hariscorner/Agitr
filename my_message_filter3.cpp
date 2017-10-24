#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include<geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iomanip>	//for std::setprecision and std::fixed

using namespace message_filters;

void callback(const geometry_msgs::Twist::ConstPtr& velIn, const turtlesim::Pose::ConstPtr& msg)
{
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "lin_vel=(" << velIn->linear.x << "," << velIn->linear.y << ")"
	//<< "_direction=" << velIn->angular.z );
	
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "position=(" << msg->x << "," << msg->y << ")"
	//<< "_direction=" << msg->theta << "__linear_velocity:" 
	//<< msg->linear_velocity << "___ang_vel:" << msg->angular_velocity);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "border_manager");

  ros::NodeHandle nh;

  message_filters::Subscriber<geometry_msgs::Twist> turtle_vel(nh, "velIn", 1);
  message_filters::Subscriber<turtlesim::Pose> turtle_pose(nh, "msg", 1);
  TimeSynchronizer<geometry_msgs::Twist, turtlesim::Pose> sync(turtle_vel, turtle_pose, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));	

  ros::spin();

  return 0;
}