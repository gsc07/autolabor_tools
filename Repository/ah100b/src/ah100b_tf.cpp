#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

void tfCallback(const sensor_msgs::Imu& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu_driver"));
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ah100b_tf");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/imu", 1, &tfCallback);
  ros::spin();
  return 0;
}
