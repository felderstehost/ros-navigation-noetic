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
#ifndef NAV_CORE_BASE_LOCAL_PLANNER_H
#define NAV_CORE_BASE_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace nav_core {
  /**
   * @class BaseLocalPlanner
   * @brief 提供局部路径规划的接口，navigation stack调用的所有局部路径规划器插件都要实现这个接口.
   */
  class BaseLocalPlanner{
    public:
      /**
       * @brief  根据当前位置和朝向，机器人速度，计算下发给底座的速度命令
       * @param cmd_vel 存储者下发的速度命令
       * @return True 表示找到有效速度命令
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0;

      /**
       * @brief  检查是否已经到达了目标点
       * @return True 代表已经到达
       */
      virtual bool isGoalReached() = 0;

      /**
       * @brief  更新要跟随的全局路径
       * @param plan 更新的全局路径
       * @return True 代表更新成功
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief  构造局部规划器
       * @param name 局部规划器实例名
       * @param tf transform listener的指针
       * @param costmap_ros 指向代价地图ROS封装类的指针
       */
      virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  接口的虚析构
       */
      virtual ~BaseLocalPlanner(){}

    protected:
      BaseLocalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_LOCAL_PLANNER_H
