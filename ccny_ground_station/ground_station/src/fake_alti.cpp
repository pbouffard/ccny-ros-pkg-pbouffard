#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  geometry_msgs::Pose pose;
  double alt=0;
  ros::init(argc, argv, "fake_alti");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("fake_alti", 1);
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
	 alt=alt+1.111;
    pose.position.z=alt;
    chatter_pub.publish(pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
