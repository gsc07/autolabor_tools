/***************************************************************
 *
 * by Jackey-Huo
 * 2017/08/23
 *
 * ************************************************************/

#ifndef ROTATE_FIRST_COST_FUNCTION_H_
#define ROTATE_FIRST_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

/**
 * This class will compare robot current direction with path
 * direction, if the different is small enough, it'll be ignored,
 * else it with score the trajectory to make the robot turn into
 * right direction first
 */
class RotateFirstCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  RotateFirstCostFunction();
  ~RotateFirstCostFunction();

  void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

  bool prepare() { return true; }

  double scoreTrajectory(Trajectory &traj);

  double getThreshold() { return angle_threshold_; }
  void setAngleThreshold(double angle_threshold) { angle_threshold_ = angle_threshold; }

private:
  bool getAngleToPath(double& angle);

  bool isNeedRotate(double angle);

  // parameter
  double                                       angle_threshold_;
  double                                       dist_threshold_;
  double                                       max_xv_;
  double                                       cost_factor_;
  double                                       thetav_bias_;
  std::string                                  global_frame_;
  std::string                                  robot_base_frame_;


  std::vector<geometry_msgs::PoseStamped>      target_poses_;
  tf::TransformListener                        tf_;
  ros::NodeHandle                              nh_;

};

}



















#endif
