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
#ifndef TRAJECTORY_ROLLOUT_COSTMAP_MODEL_
#define TRAJECTORY_ROLLOUT_COSTMAP_MODEL_

#include <base_local_planner/world_model.h>
// For obstacle data access
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {
  /**
   * @class CostmapModel
   * @brief A class that implements the WorldModel interface to provide grid
   * based collision checks for the trajectory controller using the costmap.
   */
  class CostmapModel : public WorldModel {
    public:
      /**
       * @brief  构造代价地图模型
       * @param costmap 用到的代价函数
       * @return
       */
      CostmapModel(const costmap_2d::Costmap2D& costmap);

      /**
       * @brief  世界模型的析构函数
       */
      virtual ~CostmapModel(){}
      using WorldModel::footprintCost;

      /**
       * @brief  检查是否有障碍物落到凸footprint里面，其中footprint已经被栅格化成网格
       * @param  position 机器人位置
       * @param  footprint  机器人的footprint
       * @param  inscribed_radius  机器人的内切圆半径
       * @param  circumscribed_radius  机器人的外接圆半径
       * @return 返回正数如果所有的点都在footprint外部，负数则分别代表:
       *            -1 代表 footprint 碰到至少一个致命的障碍物单元格(lethal obstacle cell)
       *            -2  代表 footprint 碰到至少一个无信息单元格 (no-information cell)
       *            -3 代表 footprint 有一部分在地图外面
       */
      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      /**
       * @brief   栅格化一条线段到代价地图网格并且进行碰撞检测
       * @param x0  网格坐标系中第一个单元格的x坐标
       * @param y0  网格坐标系中第一个单元格的y坐标
       * @param x1  网格坐标系中第二个单元格的x坐标
       * @param y1  网格坐标系中第二个单元格的y坐标
       * @return  对于合法的线段返回正值的代价
       */
      double lineCost(int x0, int x1, int y0, int y1) const;

      /**
       * @brief  检查某个点在代价地图中的代价
       * @param x 点在单元格坐标系中的x坐标
       * @param y  点在单元格坐标系中的y坐标
       * @return 对于合法的线段返回正值的代价
       */
      double pointCost(int x, int y) const;

    private:
      const costmap_2d::Costmap2D& costmap_; ///< @brief Allows access of costmap obstacle information

  };
};
#endif
