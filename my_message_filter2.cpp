#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include<geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iomanip>	//for std::setprecision and std::fixed

using namespace message_filters;

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  // make changes here
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "border_manager");

  ros::NodeHandle nh;

  message_filters::Subscriber<geometry_msgs::Twist> turtle_vel(nh, "vel", 1);
  message_filters::Subscriber<turtlesim::Pose> turtle_pose(nh, "pose", 1);
  TimeSynchronizer<geometry_msgs::Twist, turtlesim::Pose> sync(turtle_vel, turtle_pose, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}