/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <base_local_planner/prefer_forward_cost_function.h>

#include <math.h>

namespace base_local_planner {


double PreferForwardCostFunction::scoreTrajectory(Trajectory &traj) {
  // 如果机器人没有看向后面的传感器，强烈不建议后退
  if (traj.xv_ < 0.0) {
    return penalty_;
  }
  // 惩罚线速度和角速度都很小的运动
  if (traj.xv_ < 0.1 && fabs(traj.thetav_) < 0.2) {
    return penalty_;
  }
  // 旋转的越多，前进的越少
  return fabs(traj.thetav_) * 10;
}

} /* namespace base_local_planner */
