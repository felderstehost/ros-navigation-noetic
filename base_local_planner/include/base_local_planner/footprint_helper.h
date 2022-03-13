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

#ifndef FOOTPRINT_HELPER_H_
#define FOOTPRINT_HELPER_H_

#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <base_local_planner/Position2DInt.h>

namespace base_local_planner {

class FootprintHelper {
public:
  FootprintHelper();
  virtual ~FootprintHelper();

  /**
   * @brief  获得组成机器人footprint的单元格
   * @param x_i 机器人的x坐标
   * @param y_i  机器人的y坐标
   * @param theta_i 机器人的朝向
   * @param  fill 如果为 true: 返回机器人footprint的所有单元格. 如果false: 只返回组成机器人轮廓的单元格.
   * @return 返回组成机器人footprint的所有单元格/轮廓的单元格
   */
  std::vector<base_local_planner::Position2DInt> getFootprintCells(
      Eigen::Vector3f pos,
      std::vector<geometry_msgs::Point> footprint_spec,
      const costmap_2d::Costmap2D&,
      bool fill);

  /**
   * @brief  Bresenham's 算法追踪网格中连接两点的线
   * @param  x0  第一个点的x坐标
   * @param  x1  第二个点的x坐标
   * @param  y0  第一个点的y坐标
   * @param  y1  第二个点的y坐标
   * @param  pts Will be filled with the cells that lie on the line in the grid
   */
  void getLineCells(int x0, int x1, int y0, int y1, std::vector<base_local_planner::Position2DInt>& pts);

  /**
   * @brief  填充多边形的轮廓，即机器人的footprint
   * @param footprint  组成footprint的单元格集合
   */
  void getFillCells(std::vector<base_local_planner::Position2DInt>& footprint);
};

} /* namespace base_local_planner */
#endif /* FOOTPRINT_HELPER_H_ */
