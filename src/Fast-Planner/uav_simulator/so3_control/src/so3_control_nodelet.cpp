#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <so3_control/SO3Control.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"//
//HTQR

#include <plan_env/edt_environment.h>

class SO3ControlNodelet : public nodelet::Nodelet
{
public:
  SO3ControlNodelet()
    : position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_dot_(0)
    , current_yaw_(0)
    , enable_motors_(true)
    , // FIXME
    use_external_yaw_(false)
  {
  }

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void position_cmd_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  //HTQR
   void body_odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg);
  void imu_callback(const sensor_msgs::Imu& imu);
  //void SimsetEnvironment(const EDTEnvironment::Ptr& env) {
 // this->edt_environment_ = env;
//}

  SO3Control      controller_;
  ros::Publisher  so3_command_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber body_odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber enable_motors_sub_;
  ros::Subscriber corrections_sub_;
  ros::Subscriber imu_sub_;

  bool        position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double          des_yaw_, des_yaw_dot_;
  double          des_roll_, des_roll_dot_;//HTQR
  double          current_roll_;//HTQR
  double          current_yaw_;
  bool            enable_motors_;
  bool            use_external_yaw_;
  double          kR_[3], kOm_[3], corrections_[3];
  //fast_planner::EDTEnvironment::Ptr edt_environment_;
};

void
SO3ControlNodelet::publishSO3Command(void)
{//des_roll_---> orientation_b 
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,des_roll_,//HTQR
                               des_yaw_dot_, kx_, kv_);//微分平坦 计算得到角度orientation_ orientation_b 

  const Eigen::Vector3d&    force       = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();
  //orientation_b -->orientationb
  const Eigen::Quaterniond& orientationb = controller_.getComputedOrientationb();
  //std::cout<<"Quaterniond& orientationb "<<orientationb.x()<<std::endl;  right

  quadrotor_msgs::SO3Command::Ptr so3_command(
    new quadrotor_msgs::SO3Command); //! @note memory leak?
  so3_command->header.stamp    = ros::Time::now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x         = force(0);
  so3_command->force.y         = force(1);
  so3_command->force.z         = force(2);
  so3_command->orientation.x   = orientation.x();
  so3_command->orientation.y   = orientation.y();
  so3_command->orientation.z   = orientation.z();
  so3_command->orientation.w   = orientation.w();
  //借用数据位
   //orientationb.x -->so3_cmd.orientationb.x 
  so3_command->orientationb.x   = orientationb.x();

  for (int i = 0; i < 3; i++)
  {
    so3_command->kR[i]  = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  so3_command->aux.current_yaw          = current_yaw_;
  so3_command->aux.kf_correction        = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors        = enable_motors_;
  so3_command->aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_.publish(so3_command);//发布command right
}



void
SO3ControlNodelet::position_cmd_callback(//HTQR 2
  const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  des_roll_             = cmd->roll;
  des_roll_dot_         = cmd->roll_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;
 // std::cout<<"des_roll_"<<des_roll_<<std::endl;

  publishSO3Command();
}

void
SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)
      publishSO3Command();
    position_cmd_updated_ = false;
  }



}

//HTQR
void
SO3ControlNodelet::body_odom_callback(const nav_msgs::Odometry::ConstPtr& odom) 
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);
  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
  
  Eigen::Quaterniond Qb(odom->pose.pose.orientation.w,
  odom->pose.pose.orientation.x,
  odom->pose.pose.orientation.y,
  odom->pose.pose.orientation.z);
  Eigen::Matrix3d Rb;
  Rb=Qb.matrix();
  double wing_width = 0.25;
  const Eigen::Vector3d wing_right(0,-wing_width,0);
  const Eigen::Vector3d wing_left(0,wing_width,0);
  Eigen::Vector3d wing_left_esdf =   Rb*wing_left + position;
  Eigen::Vector3d wing_right_esdf =  Rb*wing_right + position;
  //获得梯度信息
  double      dist;
  Eigen::Vector3d right_grad;
  //edt_environment_->evaluateEDTWithGrad(wing_right_esdf, -1.0, dist, right_grad);
  //std::cout<<"right_grad  "<<right_grad.transpose()<<std::endl;
  //向量向heading法平面投影

  //std::cout<<"wing_left_esdf "<<wing_left_esdf.transpose()<<std::endl<<
  //"wing_right_esdf "<<wing_right_esdf.transpose()<<std::endl;
  
 // double useless_pitch;z
  //tf::Matrix3x3::Matrix3x3(odom->pose.pose.orientation).getRPY( current_roll_, useless_pitch,current_yaw_);
  //current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

//  controller_.setPosition(position);
  //controller_.setVelocity(velocity);
  //right
  //std::cout<<"current_roll_ = "<<current_roll_<<"current_yaw_  "<<current_yaw_<<std::endl;

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)
      //publishSO3Command();
    position_cmd_updated_ = false;
  }
}
void
SO3ControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void
SO3ControlNodelet::corrections_callback(
  const quadrotor_msgs::Corrections::ConstPtr& msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void
SO3ControlNodelet::imu_callback(const sensor_msgs::Imu& imu)
{
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  controller_.setAcc(acc);
}

void
SO3ControlNodelet::onInit(void)
{
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 0.5);
  controller_.setMass(mass);

  n.param("use_external_yaw", use_external_yaw_, true);

  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);

  n.param("corrections/z", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);//output HTQR 

  odom_sub_ = n.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());
                          //HTQR
  body_odom_sub_ = n.subscribe("/quadrotor_simulator_so3/body_odom_body", 10, &SO3ControlNodelet::body_odom_callback, this,
                          ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ =
    n.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback,
                this, ros::TransportHints().tcpNoDelay());//got cmd-HTQR

  enable_motors_sub_ =
    n.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                ros::TransportHints().tcpNoDelay());
  corrections_sub_ =
    n.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback,
                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = n.subscribe("imu", 10, &SO3ControlNodelet::imu_callback, this,
                         ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);