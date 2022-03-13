/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}
// 计算路径代价的函数：
//  costs为代价地图的指针，potential为代价数组，cycles为循环次数，代码里值为2*nx*ny为地图栅格数的两倍
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    // queue_为启发式搜索到的向量队列：<i , cost>
    queue_.clear();
    // 起点的索引
    int start_i = toIndex(start_x, start_y);
    // step 1 将起点放入队列
    queue_.push_back(Index(start_i, 0));
    // step 2 potential数组值全设为极大值
    // std::fill(a,b,x) 将a到b的元素都赋予x值
    std::fill(potential, potential + ns_, POT_HIGH);  // ns_ : 单元格总数
    // step 3 将起点的potential设为0
    potential[start_i] = 0;
    // 目标索引
    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;
    // 进入循环，继续循环的判断条件为只要队列大小大于0且循环次数小于所有格子数的2倍
    while (queue_.size() > 0 && cycle < cycles) {
        // step 4 得到最小cost的索引，并删除它，如果索引指向goal(目的地)则退出算法，返回true
        Index top = queue_[0];
        // step 4.1 将向量第一个元素(最小的代价的Index)和向量最后一个位置元素对调，再用pop_back删除这个元素
        // pop_heap(Iter,Iter,_Compare) _Compare有两种参数，一种是greater（小顶堆），一种是less（大顶堆）,先对调，再排序
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        // 删除最小代价的点
        queue_.pop_back();

        int i = top.i;
        // step 4.2 若是目标点则终止搜索，搜索成功
        if (i == goal_i)
            return true;
        // step 4.3 将代价最小点i周围点加入搜索队里并更新代价值, 即对前后左右四个点执行add函数
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

// 添加点并更新代价函数
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    // 超出范围， ns_为栅格总数
    if (next_i < 0 || next_i >= ns_)
        return;
    // 忽略已经搜索过的点
    if (potential[next_i] < POT_HIGH)
        return;
    // 忽略障碍物点
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;
    // p_calc_->calculatePotential() 采用简单方法计算值为costs[next_i] + neutral_cost_+ prev_potentia  地图代价+单格距离代价(初始化为50)+之前路径代价 为G
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    // 算出该点的x,y坐标
    int x = next_i % nx_, y = next_i / nx_;
    // 注意这里计算的是曼哈顿距离不是欧几里得距离
    float distance = abs(end_x - x) + abs(end_y - y);

    // potential[next_i]：    起始点到当前点的cost即g(n)
    // distance * neutral_cost_：   当前点到目的点的cost即h(n)。
    // f(n)=g(n)+h(n)：  计算完这两个cost后，加起来即为f(n)，将其存入队列中
    // 加入搜索向量
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    // 对加入的再进行堆排序， 把最小代价点放到front堆头queue_[0]
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner
