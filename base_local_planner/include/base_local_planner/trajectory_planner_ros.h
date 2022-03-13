/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/voxel_grid_model.h>
#include <base_local_planner/trajectory_planner.h>
#include <base_local_planner/map_grid_visualizer.h>

#include <base_local_planner/planar_laser_scan.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/buffer.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <base_local_planner/odometry_helper_ros.h>

namespace base_local_planner {
  /**
   * @class TrajectoryPlannerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class TrajectoryPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  默认构造函数
       */
      TrajectoryPlannerROS();

      /**
       * @brief   构造ros封装类
       * @param name 局部轨迹规划器的名字
       * @param tf  transform listener的指针
       * @param costmap 代价地图
       */
      TrajectoryPlannerROS(std::string name,
                           tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief   构造ros封装类
       * @param name 局部轨迹规划器的名字
       * @param tf  transform listener的指针
       * @param costmap 代价地图
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  封装类的析构函数
       */
      ~TrajectoryPlannerROS();

      /**
       * @brief  根据当前机器人坐标，朝向和速度，计算下个下发速度
       * @param cmd_vel 存储者下发速度
       * @return True意味着找到有效轨迹
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief 更新局部规划器要跟随的路径
       * @param orig_global_plan 更新的路径
       * @return True 表示更新成功
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      // 判断是否到达目标点
      bool isGoalReached();

      /**
       * @brief  计算轨迹并且为其打分
       * @param vx_samp 用于推算出路径的x方向速度
       * @param vy_samp 用于推算出路径的y方向速度
       * @param vtheta_samp 用于推算出路径的角速度
       * @param update_map 是否更新地图
       * when computing the legality of the trajectory, this is useful to set
       * to false if you're going to be doing a lot of trajectory checking over
       * a short period of time
       * @return True if the trajectory is legal, false otherwise
       */
      bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true);

      /**
       * @brief  产生和打分单个局部轨迹
       * @param vx_samp 用于推算出路径的x方向速度
       * @param vy_samp 用于推算出路径的y方向速度
       * @param vtheta_samp 用于推算出路径的角速度
       * @param update_map 是否更新地图
       * when computing the legality of the trajectory, this is useful to set
       * to false if you're going to be doing a lot of trajectory checking over
       * a short period of time
       * @return score of trajectory (double)
       */
      double scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true);

      bool isInitialized() {
        return initialized_;
      }

      /** @brief 返回内部的TrajectoryPlanner对象，只有在initialize()后才有效 */
      TrajectoryPlanner* getPlanner() const { return tc_; }

    private:
      /**
       * @brief  用于更新局部规划器参数的回调函数
       */
      void reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level);

      /**
       * @brief 一旦到了目标点位置上，开始旋转对齐
       * @param  global_pose 机器人在全局坐标系下的位姿
       * @param  robot_vel 机器人速度
       * @param  goal_th 期望的th 速度
       * @param  cmd_vel 速度命令的载体
       * @return  True 代表找到有效的局部路径规划
       */
      bool rotateToGoal(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  基于加速度限制让机器人停止
       * @param  global_pose 机器人在全局坐标系下的位姿
       * @param  robot_vel 机器人速度
       * @param  cmd_vel 速度命令的载体
       * @return   True 代表找到有效的局部路径规划
       */
      bool stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose, const geometry_msgs::PoseStamped& robot_vel, geometry_msgs::Twist& cmd_vel);

      std::vector<double> loadYVels(ros::NodeHandle node);

      double sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
      }

      WorldModel* world_model_; // 局部规划器用到的世界模型
      TrajectoryPlanner* tc_; ///< @brief 轨迹局部规划器，tc是 trajectory controller的缩写

      // 局部规划器使用的关于代价地图的ROS封装类
      costmap_2d::Costmap2DROS* costmap_ros_;
      // 局部规划器用到的代价地图
      costmap_2d::Costmap2D* costmap_;

      // 地图网格可视化代价地图产生的势场
      MapGridVisualizer map_viz_;
      // 用于点云转换
      tf2_ros::Buffer* tf_;
      // 全局坐标系
      std::string global_frame_;
      // 传感器的有效感知距离
      double max_sensor_range_;
      // 用于得到机器人的速度
      nav_msgs::Odometry base_odom_;
      // 机器人的基坐标系
      std::string robot_base_frame_;
      double rot_stopped_velocity_, trans_stopped_velocity_;
      double xy_goal_tolerance_, yaw_goal_tolerance_, min_in_place_vel_th_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      bool prune_plan_;
      boost::recursive_mutex odom_lock_;

      double max_vel_th_, min_vel_th_;
      double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
      double sim_period_;
      bool rotating_to_goal_;
      bool reached_goal_;
      bool latch_xy_goal_tolerance_, xy_tolerance_latch_;

      ros::Publisher g_plan_pub_, l_plan_pub_;

      dynamic_reconfigure::Server<BaseLocalPlannerConfig> *dsrv_;
      base_local_planner::BaseLocalPlannerConfig default_config_;
      bool setup_;


      bool initialized_;
      base_local_planner::OdometryHelperRos odom_helper_;

      std::vector<geometry_msgs::Point> footprint_spec_;
  };
};
#endif
