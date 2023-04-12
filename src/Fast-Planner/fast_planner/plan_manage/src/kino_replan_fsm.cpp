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




#include <ostream>
#include <plan_manage/kino_replan_fsm.h>
#include "std_msgs/Float64MultiArray.h"


namespace fast_planner {


void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;//初始化标志位状态

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);//launch文件写了  不能删除
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback 设置执行时间间隔和安全检测时间间隔*/
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);
 //HTQR
  roll_check_timer = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::roll_CheckCallback, this);
 
  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);
  body_odom_sub_ = nh.subscribe("/quadrotor_simulator_so3/body_odom_body", 1, &KinoReplanFSM::bodyodometryCallback, this);

  
 


  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  left_right_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/HTQR/left_right", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;//trigger标志 开始规划
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {//点击目标点
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;//两种导航类型
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();//可视化
  have_target_ = true;//完成存储和传递目标信息

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");//“trig”为了打印信息
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}
void KinoReplanFSM::bodyodometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
  body_odom_pos_(0) = msg->pose.pose.position.x;
  body_odom_pos_(1) = msg->pose.pose.position.y;
  body_odom_pos_(2) = msg->pose.pose.position.z;

  body_odom_orient_.w() = msg->pose.pose.orientation.w;
  body_odom_orient_.x() = msg->pose.pose.orientation.x;
  body_odom_orient_.y() = msg->pose.pose.orientation.y;
  body_odom_orient_.z() = msg->pose.pose.orientation.z;
}
void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;//改变状态
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {//10Hz
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {//1s  打印异常状态
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: 
    
    {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;//改变状态并跳出
    }

    case WAIT_TARGET: 
    
    {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
    //触发信号有 odom trigger target 

    case GEN_NEW_TRAJ: 
    {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
//四元数  旋转矩阵
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();//调用重规划
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");//成功则进入 执行新轨迹阶段
        //成功会发布B样条轨迹
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");//失败则留在元状态
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      //重规划阈值
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();//当前时间距离起始时间
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
 

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");//当前时间到轨迹的起始时间t_cur 大于这个轨迹的执行时间 需要重新规划
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl; //当前距离与重点距离太小 不需要重规划
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // cout << "near start" << endl; 当前距离与起点的距离太小 不需要重规划
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {//重规划阶段
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];
      //利用当前的pvayaw

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);//

      bool success = callKinodynamicReplan();//调用规划回调函数
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");//规划成功进入执行状态
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");//失败进入生成状态
      }
      break;
    }
  }
}
void KinoReplanFSM::roll_CheckCallback(const ros::TimerEvent& e){
  LocalTrajData* info = &planner_manager_->local_data_;
  
  Eigen::Matrix3d Rb= body_odom_orient_.toRotationMatrix();


   double wing_width = 1.0;
  const Eigen::Vector3d wing_right(0.05,-wing_width,0);
  const Eigen::Vector3d wing_left(0.05,wing_width,0);

  Eigen::Vector3d wing_right_esdf =   Rb*wing_right + body_odom_pos_;
  Eigen::Vector3d wing_left_esdf =   Rb*wing_left + body_odom_pos_;


  double right_dist;
  double left_dist;
  Eigen::Vector3d right_grad;
  Eigen::Vector3d left_grad;

  auto edt_env = planner_manager_->edt_environment_;
  edt_env->evaluateEDTWithGrad(wing_right_esdf, -1.0, right_dist, right_grad);
  edt_env->evaluateEDTWithGrad(wing_left_esdf, -1.0, left_dist, left_grad);


  //梯度在截面的投影
  Eigen::Vector3d right_grad_proj(0,0,0);
  Eigen::Vector3d left_grad_proj(0,0,0);
  Eigen::Vector3d rot_y =Rb.block(0, 1, 3, 1);
  Eigen::Vector3d rot_z =Rb.block(0, 2, 3, 1);
  right_grad_proj(1) = right_grad.dot(rot_y);
  right_grad_proj(2) = right_grad.dot(rot_z);
  
  left_grad_proj(1) = left_grad.dot(rot_y);
  left_grad_proj(2) = left_grad.dot(rot_z);

    if (right_dist>-0.001||abs(right_grad_proj(1))<0.01)//left_dist>0&&
  {
    //roll_cmd_fsm = 0;
    std::cout<<"right_grad:="<<right_grad_proj<<std::endl;
    std::cout<<"right_dist:="<<right_dist<<std::endl;
  }
  else if(abs(atan(right_grad_proj(2)/right_grad_proj(1)))>1.2)//vertical
  {
    double roll_ref = - atan(right_grad_proj(2)/right_grad_proj(1));
    double right_roll = acos(1-abs(right_dist)/wing_width);
   // double left_roll = acos(1-abs(left_dist)/wing_width);
    //roll_cmd_fsm = max(right_roll,left_roll);
    roll_cmd_fsm = right_roll;
   
    std::cout<<"2222"<<std::endl;
    std::cout<<"###roll_cmd_fsm = "<<roll_cmd_fsm<<std::endl;

    
  }else
  {
     double roll_ref =  - atan(right_grad_proj(2)/right_grad_proj(1));
      roll_cmd_fsm =  0;//roll_ref;
      std::cout<<"33333"<<std::endl;
      std::cout<<"###roll_cmd_fsm = "<<roll_cmd_fsm<<std::endl;
  }
  
  
//-----pub---//
  std_msgs::Float64MultiArray msg;
  msg.data.push_back(wing_left_esdf(0));
  msg.data.push_back(wing_left_esdf(1));
  msg.data.push_back(wing_left_esdf(2));

  msg.data.push_back(wing_right_esdf(0));
  msg.data.push_back(wing_right_esdf(1));
  msg.data.push_back(wing_right_esdf(2));

  msg.data.push_back(roll_cmd_fsm);

  
  left_right_pub_.publish(msg);
  //std::cout<<"right_grad:="<<wing_right_esdf<<std::endl;
   //写上1会报错

}
void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =//start是当前的点 end是点击屏幕的点
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);//成功会发布B样条轨迹

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
