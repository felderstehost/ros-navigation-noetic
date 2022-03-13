/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner/dwa_planner.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief DWAPlannerROS是封装类，提供了与move_base的接口，作为move_base的插件
   */
  class DWAPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief DWAPlannerROS 封装类的构造
       */
      DWAPlannerROS();

      /**
       * @brief  构建ros的封装
       * @param name 局部规划器名字
       * @param tf transform listener 的指针
       * @param costmap 代价地图
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  封装类的析构
       */
      ~DWAPlannerROS();

      /**
       * @brief  计算下发速度根据机器人的位姿，当前速度和目标点
       * @param cmd_vel 下发速度
       * @return True:找到有效路径
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


      /**
       * @brief  根据机器人的位姿，当前速度和目标点，用动态窗口方法计算下发速度
       * @param cmd_vel 下发速度
       * @return True:找到有效路径
       */
      bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  更新全局路径
       * @param orig_global_plan 传递给局部规划器的全局路径
       * @return True: 更新成功
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  是否到达目标点
       * @return True : 到达目标点
       */
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }

    private:
      /**
       * @brief 动态更新局部规划器参数的回调函数
       */
      void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      tf2_ros::Buffer* tf_; ///< @brief 用于点云转换

      // 全局和局部路径的可视化
      ros::Publisher g_plan_pub_, l_plan_pub_;

      // 用来存储运动控制参数以及costmap2d、tf等，会被传入dp_
      base_local_planner::LocalPlannerUtil planner_util_;
      // 指向dwa局部路径规划器类的共享指针
      boost::shared_ptr<DWAPlanner> dp_;

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
      dwa_local_planner::DWAPlannerConfig default_config_;
      bool setup_;
      geometry_msgs::PoseStamped current_pose_;
      // 到达目标点后的停止然后旋转的运动控制类
      base_local_planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;

      // 用来辅助获取odom信息，会被传入dp_
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
  };
};
#endif
