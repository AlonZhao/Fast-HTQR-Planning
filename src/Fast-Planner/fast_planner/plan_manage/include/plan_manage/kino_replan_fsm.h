/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {


class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  ros::NodeHandle nh_;

public:
  Test(const int& v) {
    test_ = v;
  }
  Test(ros::NodeHandle& node) {
    nh_ = node;
  }
  ~Test() {
  }
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_,body_odom_pos_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  Eigen::Quaterniond body_odom_orient_;
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;
  double roll_cmd_fsm ;//HTQR

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_, roll_check_timer;
  ros::Subscriber waypoint_sub_, odom_sub_,body_odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;
  ros::Publisher left_right_pub_; 

  /* helper functions */
  bool callKinodynamicReplan();        // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions  主要的回调函数*/
  void execFSMCallback(const ros::TimerEvent& e);//包含上次和本次的期望回调时间 实际回调时间
  void checkCollisionCallback(const ros::TimerEvent& e);//碰撞检测
  //HTQR
  void roll_CheckCallback(const ros::TimerEvent& e);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);//包含位置和姿态 获取目标点 触发回调函数 设置状态机
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);//包含位置姿态以及协方差  感知数据的信息 设置标志位
  void bodyodometryCallback(const nav_msgs::OdometryConstPtr& msg);
public:
  KinoReplanFSM(/* args */) {
    roll_cmd_fsm = 0;


  }
  ~KinoReplanFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif