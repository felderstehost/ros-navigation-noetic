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
#ifndef NAV_CORE_BASE_GLOBAL_PLANNER_H
#define NAV_CORE_BASE_GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace nav_core {
  /**
   * @class BaseGlobalPlanner
   * @brief 提供全局路径规划的接口. navigation stack调用的所有全局路径规划器插件都要实现这个接口.
   */
  class BaseGlobalPlanner{
    public:
      /**
       * @brief 已知世界坐标系下的目标点，计算全局路径
       * @param start 起始位姿
       * @param goal 目标位姿
       * @param plan 被填充的全局路径
       * @return True 表示找到有效路径
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief 已知世界坐标系下的目标点，计算全局路径
       * @param start 起始位姿
       * @param goal 目标位姿
       * @param plan 被填充的全局路径
       * @param cost 该全局路径的代价
       * @return True 表示找到有效路径
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                            double& cost)
      {
        cost = 0;
        return makePlan(start, goal, plan);
      }

      /**
       * @brief  BaseGlobalPlanner 的初始化函数
       * @param  name 全局规划器名字
       * @param  costmap_ros 指向代价地图ROS封装类的指针
       */
      virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  虚析构
       */
      virtual ~BaseGlobalPlanner(){}

    protected:
      BaseGlobalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_GLOBAL_PLANNER_H