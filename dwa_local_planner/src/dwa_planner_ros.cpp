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

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // ??????????????????????????????????????????
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // ??????dwa???????????????
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }

  // ????????????????????????
  void DWAPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // ????????????????????????????????????
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      // ?????????????????????????????????????????????????????????????????????????????? dp_?????????DWAPlanner??????shared_ptr
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }

      initialized_ = true;

      // ????????????????????????????????????????????????
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  // ??????????????????????????????
  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // ???????????????????????????????????????latch
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS(){
    // ??????new???,????????????????????????delele
    delete dsrv_;
  }



  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {
    // ???????????????????????????????????????
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    // OdometryHelperRos????????????getRobotVel??????????????????????????????
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    // ??????????????????????????????
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

    // call with updated footprint
    // ??????footprint
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment ???????????????????????????????????????????????????
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    // ???????????????????
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //  ??????????????????????????????????????????log
    // ??????????????????????????????????????????????????????
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // ???????????????????????????
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }

    // ???????????????????????????
    publishLocalPlan(local_plan);
    return true;
  }



  // ??????????????????????????????,??????????????????????????????????????????
  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    // ????????????????????????
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    // planner_util_.getLocalPlan?????????????????????????????????????????????base_local_planner???local_planner_util.cpp???
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    // ?????????????????????????????????????????????
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // ???dwa_planner?????????????????????(????????????????????????????????????)?????????????????????
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    // ???????????????????????????LatchedStopRotateController?????????????????????????????????base_local_planner???
    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      // ???????????????????????????????????????????????????
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      // ??????????????????
      publishGlobalPlan(transformed_plan);
      // ??????????????????
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
    } else {
      // ?????????????????????????????????dwaComputeVelocityCommands??????????????????????????????????????????
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }


};
