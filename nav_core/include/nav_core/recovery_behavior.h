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
#ifndef NAV_CORE_RECOVERY_BEHAVIOR_H
#define NAV_CORE_RECOVERY_BEHAVIOR_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace nav_core {
  /**
   * @class RecoveryBehavior
   * @brief 提供恢复行为的接口，navigation stack调用的所有恢复行为模块都要实现这个接口.
   */
  class RecoveryBehavior{
    public:
      /**
       * @brief  RecoveryBehavior 的初始化函数
       * @param tf transform listener 的指针
       * @param global_costmap 指向 global_costmap 的指针
       * @param local_costmap 指向 local_costmap 的指针
       */
      virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap) = 0;

      /**
       * @brief   运行RecoveryBehavior
       */
      virtual void runBehavior() = 0;

      /**
       * @brief  接口的虚析构
       */
      virtual ~RecoveryBehavior(){}

    protected:
      RecoveryBehavior(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_RECOVERY_BEHAVIOR_H
