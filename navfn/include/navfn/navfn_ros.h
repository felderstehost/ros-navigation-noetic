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
#ifndef NAVFN_NAVFN_ROS_H_
#define NAVFN_NAVFN_ROS_H_

#include <ros/ros.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/potarr_point.h>

namespace navfn {
  /**
   * @class NavfnROS
   * @brief  Navfn 全局规划器的ROS封装类
   */
  class NavfnROS : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief NavFnROS对象的默认构造函数
       */
      NavfnROS();

      /**
       * @brief  NavFnROS对象的构造函数
       * @param  name 全局规划器名字
       * @param  costmap 代价地图的指针
       */
      NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  还是NavFnROS对象的构造函数
       * @param  name 全局规划器名字
       * @param  costmap  代价地图的指针
       * @param  global_frame 代价地图的全局坐标系
       */
      NavfnROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      /**
       * @brief  NavFnROS 对象的初始化函数
       * @param  name 全局规划器名字
       * @param  costmap 代价地图的指针
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  avFnROS 对象的初始化函数
       * @param  name 全局规划器名字
       * @param  costmap 代价地图的指针
       * @param  global_frame 代价地图的全局坐标系
       */
      void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      /**
       * @brief  根据目标位姿计算全局路径
       * @param start 起始位姿
       * @param goal 目标位姿
       * @param plan 计算出的全局路径
       * @return True 代表找到全局路径
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief 根据目标位姿计算全局路径
       * @param start 起始位姿
       * @param goal 目标位姿
       * @param tolerance 目标点的容忍误差
       * @param plan 计算出的全局路径
       * @return True 代表找到全局路径
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  计算全局地图的导航函数，从已知的起始点到目标点
       * @param world_point The point to use for seeding the navigation function
       * @return True 代表 navigation function 成功的计算出来
       */
      bool computePotential(const geometry_msgs::Point& world_point);

      /**
       * @brief 当起始点的potential被计算出来后，计算全局路径  (注意: 要先调用 computePotential)
       * @param goal 目标点位姿
       * @param plan 计算出的全局路径
       * @return True 代表找到全局路径
       */
      bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief 获得 potential 或者 导航的代价  (注意: 要先调用 computePotential)
       * @param world_point 被检查的点
       * @return 该点在navigation function的值 （全局坐标系下）
       */
      double getPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief 检查potential值是否有效 (注意: 要先调用 computePotential)
       * @param world_point 被检查的点
       * @return True 代表 navigation function 在该点是有效的
       */
      bool validPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief 检查potential值是否有效 (注意: 要先调用 computePotential)
       * @param world_point 被检查的点
       * @param tolerance  在world_point附近搜索的容忍误差
       * @return True 代表 navigation function 在该点是有效的
       */
      bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

      /**
       * @brief 可视化路径
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~NavfnROS(){}

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

      /**
       * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
       */
      costmap_2d::Costmap2D* costmap_;
      boost::shared_ptr<NavFn> planner_;
      ros::Publisher plan_pub_;
      ros::Publisher potarr_pub_;
      bool initialized_, allow_unknown_, visualize_potential_;


    private:
      inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        return dx*dx +dy*dy;
      }

      void mapToWorld(double mx, double my, double& wx, double& wy);
      void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
      double planner_window_x_, planner_window_y_, default_tolerance_;
      boost::mutex mutex_;
      ros::ServiceServer make_plan_srv_;
      std::string global_frame_;
  };
};

#endif
