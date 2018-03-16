#include <base_local_planner/rotate_first_cost_function.h>
#include <ros/ros.h>

#include <cmath>

namespace base_local_planner {

RotateFirstCostFunction::RotateFirstCostFunction() :
  tf_(), nh_() {
  nh_.param("angle_threshold", angle_threshold_, 2.0);
  nh_.param("dist_threshold", dist_threshold_, 0.2);
  nh_.param("max_xv", max_xv_, 0.07);
  nh_.param("cost_factor", cost_factor_, 10.0);
  nh_.param("thetav_bias", thetav_bias_, 0.2);
  ROS_INFO("RotateFirstCostFunction angle_threshold_: %lf, dist_threshold_: %lf, max_xv_: %lf, cost_factor_: %lf, thetav_bias_: %lf", angle_threshold_, dist_threshold_, max_xv_, cost_factor_, thetav_bias_);

  ros::param::get( "/move_base/global_costmap/global_frame", global_frame_ );
  ros::param::get( "/move_base/global_costmap/robot_base_frame", robot_base_frame_ );
  ROS_INFO("RotateFirstCostFunction global_frame_: %s", global_frame_.c_str());
  ROS_INFO("RotateFirstCostFunction robot_base_frame_: %s", robot_base_frame_.c_str());
}

RotateFirstCostFunction::~RotateFirstCostFunction() { }


bool RotateFirstCostFunction::getAngleToPath(double& angle) {
  tf::StampedTransform transform;
  try {
    tf_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    ROS_DEBUG_NAMED("RotateFirstCostFunction", "fail to get transform from %s to %s, %s",
                              global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    return false;
  }

  if (!(target_poses_.size() >= 2)) {
    ROS_DEBUG_NAMED("RotateFirstCostFunction", "target_poses_ size smaller than 2, please set valid target pose first");
    return false;
  }

  double robot_angle;
  double path_angle;
  robot_angle = tf::getYaw( transform.getRotation() );
  geometry_msgs::Point p0 = target_poses_[0].pose.position;
  geometry_msgs::Point p1 = target_poses_[1].pose.position;
  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  path_angle = acos( dx / sqrt(dx*dx + dy*dy) );
  path_angle = dy > 0 ? path_angle : -path_angle;

  double diff_angle = path_angle - robot_angle;
  if ( fabs(diff_angle) > M_PI ) {
      double diff_angle_ = 2.0*M_PI - fabs(diff_angle);
      diff_angle = diff_angle > 0 ? diff_angle_ : -diff_angle_;
  }

  angle = diff_angle;
  return true;
}

void RotateFirstCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}

bool RotateFirstCostFunction::isNeedRotate(double angle) {
  geometry_msgs::Point path0 = target_poses_[0].pose.position;
  tf::StampedTransform transform;
  tf_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(0), transform);
  tf::Vector3 t_robot = transform.getOrigin();

  double r_x = t_robot.x(), r_y = t_robot.y();
  double p_x = path0.x, p_y = path0.y;
  double dist = sqrt( (r_x-p_x)*(r_x-p_x) + (r_y-p_y)*(r_y-p_y) );

  if ( dist < dist_threshold_ && fabs(angle) > angle_threshold_ )
      return true;
  else
  {
    return false;
  }
}

double RotateFirstCostFunction::scoreTrajectory(Trajectory &traj) {

  double angle_to_path;

  if ( !getAngleToPath(angle_to_path) ) {
    ROS_ERROR_NAMED("RotateFirstCostFunction",
        "fail to get robot angle to map, please start debug mode to get more details");
  }

  if (!isNeedRotate(angle_to_path))
  {
    return 0;
  }
  else
      ROS_DEBUG("RotateFirstCostFunction begin work");

  if ( fabs(traj.xv_) > max_xv_ )
      return -1.0;
  if ( traj.thetav_*angle_to_path < 0 )
      return -1.0;

  double cost = 1.0 / (fabs(traj.thetav_) + thetav_bias_) * cost_factor_;

  return cost;
}


}
