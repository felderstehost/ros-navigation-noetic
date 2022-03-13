/*
 * latched_stop_rotate_controller.h
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#ifndef LATCHED_STOP_ROTATE_CONTROLLER_H_
#define LATCHED_STOP_ROTATE_CONTROLLER_H_

#include <string>

#include <Eigen/Core>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>

namespace base_local_planner {

// 当机器人里目标点足够近的时候，LatchedStopRotateController控制器会被调用
class LatchedStopRotateController {
public:
  LatchedStopRotateController(const std::string& name = "");
  virtual ~LatchedStopRotateController();

  bool isPositionReached(LocalPlannerUtil* planner_util,
                         const geometry_msgs::PoseStamped& global_pose);

  bool isGoalReached(LocalPlannerUtil* planner_util,
      OdometryHelperRos& odom_helper,
      const geometry_msgs::PoseStamped& global_pose);

  void resetLatching() {
    xy_tolerance_latch_ = false;
  }

  /**
   * @brief 停止机器人同时考虑加速度的限制
   * @param  global_pose 机器人位姿
   * @param  robot_vel 机器人速度
   * @param  cmd_vel 下发速度
   * @return  True : 找到有效路径
   */
  bool stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& robot_vel,
      geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

  /**
   * @brief Once a goal position is reached... rotate to the goal orientation 一旦到达目标点位置，开始旋转进行角度对齐
   * @param  global_pose 机器人位置
   * @param  robot_vel 机器人速度
   * @param  goal_th 期望的朝向角度
   * @param  cmd_vel 下发速度
   * @return  True : 找到有效路径
   */
  bool rotateToGoal(const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& robot_vel,
      double goal_th,
      geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      base_local_planner::LocalPlannerLimits& limits,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

  bool computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      LocalPlannerUtil* planner_util,
      OdometryHelperRos& odom_helper,
      const geometry_msgs::PoseStamped& global_pose,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

private:
  inline double sign(double x){
    return x < 0.0 ? -1.0 : 1.0;
  }


  // latch_xy_goal_tolerance_ 是否有latch机器人的需求,
  // xy_tolerance_latch_ 状态反馈机器人是否已经到了目标区域，只需要旋转就行
  bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
  bool rotating_to_goal_;
};

} /* namespace base_local_planner */
#endif /* LATCHED_STOP_ROTATE_CONTROLLER_H_ */
