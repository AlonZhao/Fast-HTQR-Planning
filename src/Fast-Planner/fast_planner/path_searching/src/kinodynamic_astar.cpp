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

#include <path_searching/kinodynamic_astar.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{//输入起止点对应的节点信息：状态信息、栅格信息
  start_vel_ = start_v;
  start_acc_ = start_a;

  PathNodePtr cur_node = path_node_pool_[0];//取出第一个点
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;//p,v 状态
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);//终点对应体素坐标
  //启发式函数计算 当前点到终点的BVP 状态向量pv
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;//加入
  open_set_.push(cur_node);//设置后加入open表加入优先队列自动排序
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);//加入已扩展的表

  PathNodePtr neighbor = NULL;//邻近
  PathNodePtr terminate_node = NULL;//终点
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();
   // std::cout << "node top" << std::endl;
    // Terminate?
    //当前点与起始点相比是否超出了规划范围
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;//就是search horizon 距离还很远的标志
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;

    if (reach_horizon || near_end)
    {
      //如果距离出发点比较远 回溯已经搜到的点
      terminate_node = cur_node;
      retrievePath(terminate_node);//回溯路径倒退 存放在path_node_pool_
      if (near_end)//靠近终点
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);//计算直达曲线最优时间
        //如果可以会设置标志位is_shot_succ_
        computeShotTraj(cur_node->state, end_state, time_to_goal);//计算直达轨迹  
        if (init_search)//没有走远 靠近终点
          ROS_ERROR("Shot in first search loop!");
      }
    }
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        //一般而言 走出一段距离 调用这个
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else if (cur_node->parent != NULL)//
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        //
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }
    //节点扩张： 当前点还在规划范围 不靠近终点
    open_set_.pop();//弹出第一个元素 上面已经top保存了
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_acc_);//第一次的输入是000
      //0.05 0.8
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
       // cout<<"start nummmmmmmmmmmm"<<durations.size()<<std::endl; 比较多
      init_search = false;//分配了时间和输入离散化 标志位
    }
    else //加速度约束
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
          //初始的和中间的时间分配不一样 1 0.6
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
        //cout<<"middle nummmmmmmmmmmm"<<durations.size()<<std::endl; 很少
    }

    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    //从当前点扩展到邻居点
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)//输入和时间离散花
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);//邻居节点
        pro_t = cur_node->time + tau;//邻居时间

        Eigen::Vector3d pro_pos = pro_state.head(3);//

        // Check if in close set
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);//时间对应的id
        //三元运算 dyn true 
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        //如果已经存在 并且已经在close表 那就条过
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check maximal velocity  检查以下扩展节点的速度 超过的不要
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // Check not in the same voxel
        //当前点和扩展出来的点不能在一个体素栅格
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)//扩展在相同的体素就条过
            std::cout << "same" << std::endl;
          continue;
        }

        // Check safety  尤其是当tau 到 tau max时候 0.8 需要再精细以下保证着0-0.8积分过程没有碰撞而不是之检查终点
        //地图分辨率和这个相关
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)//5
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);//积分获得扩展节点的位置和速度
          pos = xt.head(3);
          if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 )
          {
            //该点碰撞 不必检查
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          //碰撞 不加入
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }
        //前面的筛选都通过
        double time_to_goal, tmp_g_score, tmp_f_score;//计算
        //计算经过该点的代价 和以后的启发函数
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent  节点剪zhi操作
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)//tmp有了
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;//遍历如果相同栅格点 才需要剪(后面)
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)//第一次扩展出五碰撞是到这里  后面的都是先驱上面
        {
          if (pro_node == NULL)//如果这个扩展出的节点在所有栅格没见过
          {
            pro_node = path_node_pool_[use_node_num_];//加入路径
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);//加入open等待扩展

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);//

            tmp_expand_nodes.push_back(pro_node);//加入临时扩展

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)//如果对应格子在open中了 返回的是之前的点
          {
            if (tmp_g_score < pro_node->g_score)//新扩展的<已经存在的
            {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;//新的状态转移出的状态
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);//最大时间
  nh.param("search/init_max_tau", init_max_tau_, -1.0);//初始最大时间
  nh.param("search/max_vel", max_vel_, -1.0);//最大速度
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);//时间权重
  nh.param("search/horizon", horizon_, -1.0);//搜索范围
  nh.param("search/resolution_astar", resolution_, -1.0);//距离分辨率0.1
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);//分配到终点的节点数量10W
  nh.param("search/check_num", check_num_, -1);
  nh.param("search/optimistic", optimistic_, true);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;//
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);//费拉里方法 启发函数对时间的导数==四次方程求根

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)//比较所有的根对应cost的大小
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;//最优时间

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)//检查碰撞
    {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;
//嵌套三次求根
  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);//10W个初始化了
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();//回溯
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
{
  /* ---------- path duration ------计算最后的估计时间---- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;//最后一段是直达时间
  PathNodePtr node = path_nodes_.back();//最后一个时间
  while (node->parent != NULL)
  {
    T_sum += node->duration;
    node = node->parent;
  }
  cout << "duration:" << T_sum << endl;

  // Calculate boundary vel and acc
  Eigen::Vector3d end_vel, end_acc;
  double t;
  if (is_shot_succ_)
  {
    t = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim)
    {
      Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  }
  else
  {
    t = path_nodes_.back()->duration;
    end_vel = node->state.tail(3);
    end_acc = path_nodes_.back()->input;
  }

  // Get point samples
  int seg_num = floor(T_sum / ts);
  seg_num = max(8, seg_num);
  ts = T_sum / double(seg_num);
  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    if (sample_shot_traj)
    {
      // samples on shot traj
      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->parent != NULL)
          t += node->duration;
      }
    }
    else
    {
      // samples on searched traj
      Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d ut = node->input;

      stateTransit(x0, xt, ut, t);

      point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector3d start_acc;
  if (path_nodes_.back()->parent == NULL)
  {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  }
  else
  {
    // input of searched traj
    start_acc = node->input;
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

}  // namespace fast_planner
