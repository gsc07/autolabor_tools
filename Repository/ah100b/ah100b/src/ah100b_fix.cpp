#include <ros/ros.h>

#include "sensor_msgs/Imu.h"
#include "tf/transform_listener.h"

void fixCallback(const sensor_msgs::Imu::ConstPtr& msg, ros::Publisher * pub_ptr)
{
//  static tf::TransformListener listener;
  sensor_msgs::Imu fix_msg;
  tf::Transform transform;
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  transform.setRotation(q);
  transform.setOrigin(tf::Vector3(0,0,0));

//  tf::StampedTransform transform;
//  listener.waitForTransform("/world", "/imu_driver", ros::Time(0), ros::Duration(3));
//  listener.lookupTransform("/world", "/imu_driver", ros::Time(0), transform);

  fix_msg.header = msg->header;

  tf::Vector3 lin_acc(transform(tf::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)));
  fix_msg.linear_acceleration.x=lin_acc.getX();
  fix_msg.linear_acceleration.y=lin_acc.getY();
  fix_msg.linear_acceleration.z=lin_acc.getZ();

  tf::Vector3 ang_vec(transform(tf::Vector3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z)));
  fix_msg.angular_velocity.x=ang_vec.getX();
  fix_msg.angular_velocity.y=ang_vec.getY();
  fix_msg.angular_velocity.z=ang_vec.getZ();

  pub_ptr->publish(fix_msg);
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ah100b_fix");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("/imu_fix", 1);
  ros::Subscriber sub = node.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(fixCallback, _1, &pub));
  ros::spin();
  return 0;
}
