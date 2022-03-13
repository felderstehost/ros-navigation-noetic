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

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {

  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

  // 获取轨迹的得分，遍历各个打分项进行打分并进行求和
  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    // 特别需要关注TrajectoryCostFunction类，
    // 对critics_,这个对象的每次的迭代的对象不同的
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;
      // 先获取对应的scale参数，如果scale为0证明该打分项无效
      if (score_function_p->getScale() == 0) {
        continue;
      }
      // 下面这个函数调用可以计算出路径的代价。注意多态
      double cost = score_function_p->scoreTrajectory(traj);
      // 如果某个打分项打分小于0，则将该负值作为路径打分项，立刻返回
      if (cost < 0) {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      // 将打分项乘以对应的scale作为最终的打分项打分
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      // 对于路径的代价需要乘以系数之后进行累加。
      traj_cost += cost;
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        // 如果traj_cost已经大于best_traj_cost了，且best_traj_cost，为了节省计算资源，停止计算。
        if (traj_cost > best_traj_cost) {
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }

  // 下面这个函数得到最优的轨迹，轨迹包含了速度
  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    // 调用各个打分项的prepare函数进行准备，对于每个打分器，由于上一步已经进行了速度采样，因此可以逐个速度产生轨迹，并进行打分，同时记录最优打分和最优轨迹
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }
    // 遍历std::vector<TrajectorySampleGenerator*>轨迹产生器列表(好像目前只有一个)进行打分，
    // 并且一旦某个产生器发现了最优轨迹则返回（该做法较为粗略，即使有多个产生器也无法得到全局最优）
    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      // 检查是否还有轨迹
      while (gen_->hasMoreTrajectories()) {
        // 生成下一个轨迹。此处调用的是轨迹生成器的函数，前面的生成轨迹的依据的采样速度已经生成
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false) {
          // TODO use this for debugging
          continue;
        }
        // 对生成的轨迹进行评分
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
        if (all_explored != NULL) {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }

        if (loop_traj_cost >= 0) {
          count_valid++;
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            // 将更优秀的cost赋值给best_traj_cost
            best_traj_cost = loop_traj_cost;
            // 将cost更低的轨迹也赋值给best_traj
            best_traj = loop_traj;
          }
        }
        count++;
        if (max_samples_ > 0 && count >= max_samples_) {
          break;
        }
      }
      // 对最终轨迹的速度和cost进行填充
      if (best_traj_cost >= 0) {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          // 对最终轨迹的点数据进行填充
          traj.addPoint(px, py, pth);
        }
      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }


}// namespace
