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
 * Author: TKruse
 *********************************************************************/

#ifndef SIMPLE_SCORED_SAMPLING_PLANNER_H_
#define SIMPLE_SCORED_SAMPLING_PLANNER_H_

#include <vector>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>

namespace base_local_planner {

/**
 * @class SimpleScoredSamplingPlanner
 * @brief 根据给定的产生器和代价函数计算一个局部轨迹.这里假设代价越小越好同时负值的代价意味着无穷大
 *
 * TrajectorySearch接口的简单和鲁棒的实现，如果想用更有效率的搜索，可以考虑search heuristics, parallel search
 */
class SimpleScoredSamplingPlanner : public base_local_planner::TrajectorySearch {
public:

  ~SimpleScoredSamplingPlanner() {}

  SimpleScoredSamplingPlanner() {}

  /**
   * 处理一系列的产生器和评分器。评分器返回代价大于0，如果小于0意味着无效路径
   * 产生器不是之前的那个，是fallback产生器，只有在先前的产生器没有找到有效路径时，他们才去计算
   * 将会用到每个产生器直到不在返回局部路径或者计数满足了要求
   * 然后重置计数并且尝试列表中的下一个
   * 如果max_samples = -1 (默认值)：每一个采样规划器会持续的调用产生器直到产生器遍历了所有的样本
   * Takes a list of generators and critics. Critics return costs > 0, or negative costs for invalid trajectories.
   * Generators other than the first are fallback generators,  meaning they only get to generate if the previous
   * generator did not find a valid trajectory.
   * Will use every generator until it stops returning trajectories or count reaches max_samples.
   * Then resets count and tries for the next in the list.
   * passing max_samples = -1 (default): Each Sampling planner will continue to call
   * generator until generator runs out of samples (or forever if that never happens)
   */
  SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples = -1);

  /**
   * runs all scoring functions over the trajectory creating a weigthed sum
   * of positive costs, aborting as soon as a negative cost are found or costs greater
   * than positive best_traj_cost accumulated
   */
  double scoreTrajectory(Trajectory& traj, double best_traj_cost);

  /**
   * Calls generator until generator has no more samples or max_samples is reached.
   * For each generated traj, calls critics in turn. If any critic returns negative
   * value, that value is assumed as costs, else the costs are the sum of all critics
   * result. Returns true and sets the traj parameter to the first trajectory with
   * minimal non-negative costs if sampling yields trajectories with non-negative costs,
   * else returns false.
   *
   * @param traj The container to write the result to
   * @param all_explored pass NULL or a container to collect all trajectories for debugging (has a penalty)
   */
  bool findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored = 0);


private:
  std::vector<TrajectorySampleGenerator*> gen_list_;
  std::vector<TrajectoryCostFunction*> critics_;

  int max_samples_;
};




} // namespace

#endif /* SIMPLE_SCORED_SAMPLING_PLANNER_H_ */
