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
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_H_

#include <vector>
#include <Eigen/Core>


#include <dwa_local_planner/DWAPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlanner
   * @brief 实现DWA算法的局部规划器类
   */
  class DWAPlanner {
    public:
      /**
       * @brief  规划器的构造
       * @param name 规划器的名字
       * @param costmap_ros 规划器的将用到的代价地图实例指针
       * @param global_frame  用到的tf frame 中的frame id
       */
      DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

      /**
       * @brief 配置局部规划器
       */
      // ?DWAPlannerConfig
      void reconfigure(DWAPlannerConfig &cfg);

      /**
       * @brief  对于位置/速度组合，轨迹是否合法
       * @param pos 机器人的位置
       * @param vel 机器人的速度
       * @param vel_samples 期望速度
       * @return 如果是True, 轨迹合法
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      /**
       * @brief 从机器人当前的位置和速度找出最好的轨迹去执行
       * @param global_pose 机器人的位置
       * @param global_vel 机器人的速度
       * @param drive_velocities 给机器人底座的速度
       * @return  返回最高得分的轨迹, cost >= 0 意味着该轨迹可以被执行
       */
      base_local_planner::Trajectory findBestPath(
          const geometry_msgs::PoseStamped& global_pose,
          const geometry_msgs::PoseStamped& global_vel,
          geometry_msgs::PoseStamped& drive_velocities);

      /**
       * @brief 在路径规划前更新代价函数
       * @param  global_pose 机器人的位置
       * @param  new_plan 新的规划路径
       * @param  footprint_spec 机器人的footprint(机器人的底盘形状)
       *
       * The obstacle cost function gets the footprint.
       * The path and goal cost functions get the global_plan
       * The alignment cost functions get a version of the global plan
       *   that is modified based on the global_pose
       */
      void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
          const std::vector<geometry_msgs::PoseStamped>& new_plan,
          const std::vector<geometry_msgs::Point>& footprint_spec);

      /**
       * @brief 获得局部规划器预期所需要的计算时间
       */
      double getSimPeriod() { return sim_period_; }

      /**
       * @brief Compute the components and total cost for a map grid cell 计算地图网格单元的代价
       * @param cx 地图网格中单元格x坐标
       * @param cy 地图网格中单元格y坐标
       * @param path_cost 路径的代价
       * @param goal_cost 到目标点距离的代价
       * @param occ_cost 单元格代价值
       * @param total_cost 总体代价，有把尺度因子考虑进去
       * @return True : 这个单元格可以通过
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * 设置新的全局路径和重置状态
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    private:
      // planner_util是规划器辅助对象， 为局部规划器提供了代价地图，坐标变换，全局规划等算法输入
      base_local_planner::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; // 刹车反应时间，避免撞到障碍物
      double path_distance_bias_, goal_distance_bias_, occdist_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_; // 计算DWA最大/最小速度用的秒数
      base_local_planner::Trajectory result_traj_;

      double forward_point_distance_;

      // 局部路径规划器的参考路径，来至于全局规划，可以可视化
      std::vector<geometry_msgs::PoseStamped> global_plan_;

      boost::mutex configuration_mutex_;
      std::string frame_id_;
      ros::Publisher traj_cloud_pub_;
      bool publish_cost_grid_pc_; // 是否发布点云消息
      bool publish_traj_pc_;

      double cheat_factor_;

      base_local_planner::MapGridVisualizer map_viz_; // 可视化代价函数产生的势场

      base_local_planner::SimpleTrajectoryGenerator generator_;
      base_local_planner::OscillationCostFunction oscillation_costs_;
      base_local_planner::ObstacleCostFunction obstacle_costs_;
      base_local_planner::MapGridCostFunction path_costs_;
      base_local_planner::MapGridCostFunction goal_costs_;
      base_local_planner::MapGridCostFunction goal_front_costs_;
      base_local_planner::MapGridCostFunction alignment_costs_;
      base_local_planner::TwirlingCostFunction twirling_costs_;

      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  };
};
#endif
