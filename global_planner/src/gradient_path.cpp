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
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) {
    gradx_ = grady_ = NULL;
}

GradientPath::~GradientPath() {

    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPath::setSize(int xs, int ys) {
    Traceback::setSize(xs, ys);
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

bool GradientPath::getPath(float* potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path) {
    std::pair<float, float> current;
    int stc = getIndex(goal_x, goal_y);

    // step xx 获得x,y 在目标点的差值和单元格总数
    float dx = goal_x - (int)goal_x;
    float dy = goal_y - (int)goal_y;
    int ns = xs_ * ys_;
    // memset是以字节为单位，初始化内存块
    memset(gradx_, 0, ns * sizeof(float));
    memset(grady_, 0, ns * sizeof(float));

    int c = 0;
    while (c++<ns*4) {
        // step xx 检查是否已经靠近起始点了
        double nx = stc % xs_ + dx, ny = stc / xs_ + dy;

        if (fabs(nx - start_x) < .5 && fabs(ny - start_y) < .5) {
            current.first = start_x;
            current.second = start_y;
            path.push_back(current);
            return true;
        }
        // step xx 检查是否有越界
        if (stc < xs_ || stc > xs_ * ys_ - xs_)
        {
            printf("[PathCalc] Out of bounds\n");
            return false;
        }

        current.first = nx;
        current.second = ny;

        //ROS_INFO("%d %d | %f %f ", stc%xs_, stc/xs_, dx, dy);

        path.push_back(current);

        bool oscillation_detected = false;
        int npath = path.size();
        if (npath > 2 && path[npath - 1].first == path[npath - 3].first
                && path[npath - 1].second == path[npath - 3].second) {
            ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
            oscillation_detected = true;
        }

        int stcnx = stc + xs_; // 正下方的点
        int stcpx = stc - xs_; // 正上方的点

        // step xx 检查当前点附近的8个点的potential
        if (potential[stc] >= POT_HIGH || potential[stc + 1] >= POT_HIGH || potential[stc - 1] >= POT_HIGH
                || potential[stcnx] >= POT_HIGH || potential[stcnx + 1] >= POT_HIGH || potential[stcnx - 1] >= POT_HIGH
                || potential[stcpx] >= POT_HIGH || potential[stcpx + 1] >= POT_HIGH || potential[stcpx - 1] >= POT_HIGH
                || oscillation_detected) {
            ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());
            //  检查8个邻点，找到最小potential的那个点
            int minc = stc;
            int minp = potential[stc];
            int st = stcpx - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stc - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stc + 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st = stcnx - 1;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp) {
                minp = potential[st];
                minc = st;
            }
            stc = minc;
            dx = 0;
            dy = 0;

            //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
            //    potential[stc], path[npath-1].first, path[npath-1].second);

            if (potential[stc] >= POT_HIGH) {
                ROS_DEBUG("[PathCalc] No path found, high potential");
                //savemap("navfn_highpot");
                return 0;
            }
        }

        // step xx 如果周围的点没有potential极大值,可以求梯度了
        else {

            // 获得当前点上下左右四个方向的梯度
            gradCell(potential, stc);
            gradCell(potential, stc + 1);
            gradCell(potential, stcnx);
            gradCell(potential, stcnx + 1);

            // 获取插值的梯度
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // 插值 x
            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // 插值 y

            // show gradients
            ROS_DEBUG(
                    "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx_[stc], grady_[stc], gradx_[stc+1], grady_[stc+1], gradx_[stcnx], grady_[stcnx], gradx_[stcnx+1], grady_[stcnx+1], x, y);

            // 检查是否有梯度为零的，有则返回
            if (x == 0.0 && y == 0.0) {
                ROS_DEBUG("[PathCalc] Zero gradient");
                return 0;
            }

            // 向正确的方向移动
            float ss = pathStep_ / hypot(x, y);
            dx += x * ss;
            dy += y * ss;

            // 检查是否有溢出
            if (dx > 1.0) {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0) {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0) {
                stc += xs_;
                dy -= 1.0;
            }
            if (dy < -1.0) {
                stc -= xs_;
                dy += 1.0;
            }

        }

        //printf("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //         potential[stc], dx, dy, path[npath-1].first, path[npath-1].second);
    }

    return false;
}

/*
 int
 NavFn::calcPath(int n, int *st)
 {
 // set up start position at cell
 // st is always upper left corner for 4-point bilinear interpolation
 if (st == NULL) st = start;
 int stc = st[1]*nx + st[0];

 // go for <n> cycles at most
 for (int i=0; i<n; i++)
 {



 }

 //  return npath;            // out of cycles, return failure
 ROS_DEBUG("[PathCalc] No path found, path too long");
 //savemap("navfn_pathlong");
 return 0;            // out of cycles, return failure
 }
 */

//  该函数对当前单元格进行梯度计算，正值表示向右和向下
float GradientPath::gradCell(float* potential, int n) {
    if (gradx_[n] + grady_[n] > 0.0)    // check this cell
        return 1.0;

    if (n < xs_ || n > xs_ * ys_ - xs_)    // 如果越界的话
        return 0.0;
    float cv = potential[n];
    float dx = 0.0;
    float dy = 0.0;

    //  如果有障碍物
    if (cv >= POT_HIGH) {
        if (potential[n - 1] < POT_HIGH)
            dx = -lethal_cost_;
        else if (potential[n + 1] < POT_HIGH)
            dx = lethal_cost_;

        if (potential[n - xs_] < POT_HIGH)
            dy = -lethal_cost_;
        else if (potential[n + xs_] < POT_HIGH)
            dy = lethal_cost_;
    }

    else  // 如果没有障碍物
    {
        // dx calc, average to sides
        if (potential[n - 1] < POT_HIGH)
            dx += potential[n - 1] - cv;
        if (potential[n + 1] < POT_HIGH)
            dx += cv - potential[n + 1];

        // dy calc, average to sides
        if (potential[n - xs_] < POT_HIGH)
            dy += potential[n - xs_] - cv;
        if (potential[n + xs_] < POT_HIGH)
            dy += cv - potential[n + xs_];
    }

    // 归一化
    float norm = hypot(dx, dy);
    if (norm > 0) {
        norm = 1.0 / norm;
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }
    return norm;
}

} //end namespace global_planner

