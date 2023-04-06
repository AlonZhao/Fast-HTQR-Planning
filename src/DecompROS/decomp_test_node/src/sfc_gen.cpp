#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include"quadrotor_msgs/PositionCommand.h"
#include <std_msgs/Float64.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
//osqp
#include"OsqpEigen/OsqpEigen.h"
//画球
#include <visualization_msgs/Marker.h>


        //目标函数二次和线性部分
Eigen::SparseMatrix<double> hessian ;
Eigen::SparseMatrix<double> linearMatrix ;  //A
Eigen::Vector3d gradient;
int NumberOfVariables ; //A矩阵的列数
int NumberOfConstraints ; //A矩阵的行数
Eigen::VectorXd lowerBound; //l
Eigen::VectorXd upperBound; //u
//具有线性约束和边界的二次最小化
/*
class Spheres{
        //存放内切圆
private:
    Vec3f sp_radius;//半径
    vec_Vec3f center_pt;//圆心点
    vec_Vec3f contact_pt;//内接点
public:
    Spheres(){ROS_INFO("Class OK");}
    Vec3f getSp_radius(){return sp_radius;}
    vec_Vec3f getCenter_pt(){return center_pt;}
    vec_Vec3f getContact_pt(){return contact_pt;}
    int GetSphereNum(){return sp_radius.size();}

};*/
class SFC_GEN{

private:
//障碍物信息
    ros::NodeHandle node;
    ros::Publisher poly_pub ;
    ros::Publisher center_pt_pub;//
    ros::Publisher roll_pub;
    ros::Subscriber occurpany_sub ;
    ros::Subscriber pose_cmd ;
    ros::Subscriber pose_predict_cmd ;

    ros::Timer sfc_gen ;
    ros::Timer sfc_visual;

    vec_Vec3f vec_obs_;
    Vec3f point_now;//当前点
    Vec3f point_pred;//预测点
Vec3f center_point ;
    std::vector<LinearConstraint3D> poly_constraints;
    vec_E<Polyhedron3D> polys;//c存放多个多面体
  //  Spheres sphs;//存放
    std::vector<double> sp_radius;//半径
    vec_Vec3f center_pt;//圆心点
    vec_Vec3f contact_pt;//内接点

    //
    visualization_msgs::Marker marker2;

    int LastNum;




    public:

        SFC_GEN(ros::NodeHandle &nh)
        {
            node=nh;
            poly_pub = node.advertise<decomp_ros_msgs::PolyhedronArray>("/sfc_polyhedron", 1, true);
            occurpany_sub = node.subscribe("/sdf_map/occupancy_inflate", 1, &SFC_GEN::cloud2Callback,this);
            pose_cmd = node.subscribe("/planning/pos_cmd", 1, &SFC_GEN::cmdHandleCallback,this);
            pose_predict_cmd = node.subscribe("/pred_position_cmd", 1, &SFC_GEN::cmd_predictHandleCallback,this);
            sfc_gen = node.createTimer(ros::Duration(0.05), &SFC_GEN::sfc_along_pathCallback,this);
            sfc_visual = node.createTimer(ros::Duration(0.01), &SFC_GEN::sfc_visualCallback,this);
            //增加姿态轨迹
            roll_pub = node.advertise<std_msgs::Float64>("/roll_cmd", 1,true);
            //marker2
         

        }
~SFC_GEN()
{
   // std::cout<<"===================="<<std::endl<<sp_radius<<std::endl;
}
        double Sphere_Cal(Vec3f path_pt,MatDNf<3> consrtaint_A,VecDf b_)//求内接球半径，输入：A b约束，原点 ；输出：最小半径
        {

            int N_planes = consrtaint_A.rows();//平面个数
            consrtaint_A.resize(N_planes,3);
            double temp = 0,radius_sphere = 10000 ;
            for(int i = 0;i<N_planes;i++)
            {
                temp = -(consrtaint_A.row(i) * path_pt - b_(i))/(consrtaint_A.row(i)).norm();
                if(radius_sphere > temp)
                {
                    radius_sphere = temp;
                }
            }
        
           // std::cout<<"#################"<<radius_sphere<<"##########"<<std::endl;
        return radius_sphere;


        }
        int Radius_Cal(Vec3f path_pt,MatDNf<3> consrtaint_A,VecDf b_)//OSQP求内接球半径，输入：A b约束，原点 ；输出：最小半径
        {

        // allocate QP problem matrices and vectores

        hessian.resize(3,3);
        hessian.insert(0,0) = -2;
        hessian.insert(1,1) = -2;
        hessian.insert(2,2) = -2;

        hessian.insert(0,1) = 0;
        hessian.insert(0,2) = 0;

        hessian.insert(1,0) = 0;
        hessian.insert(1,2) = 0;

        hessian.insert(2,1) = 0;
        hessian.insert(2,0) = 0;
        //double hessian =-1; //P or H
        //path_pt.resize(3,1);
        gradient=2*path_pt;  //f or q
        //约束不等式部分
         NumberOfVariables = 3; //A矩阵的列数
         NumberOfConstraints = consrtaint_A.rows(); //A矩阵的行数
        consrtaint_A.resize(NumberOfConstraints,3);


        linearMatrix.resize(NumberOfConstraints,3);
        for(int i =0;i<NumberOfConstraints;i++)
            for(int j=0;j<3;j++)
            linearMatrix.insert(i,j)=consrtaint_A(i,j);




        lowerBound.resize(NumberOfConstraints);
       
       // lowerBound << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
        

        upperBound.resize(NumberOfConstraints);
        b_.resize(NumberOfConstraints);
        for(int i =0;i<NumberOfConstraints;i++)
        {
            upperBound(i)=b_(i);
            lowerBound(i)=-OsqpEigen::INFTY;
        }

       //upperBound << 1, 1, 1, 0.5;
       //b_.resize(3);

    // std::cout << "hessian:" << std::endl << hessian.size()<< std::endl;
        //std::cout << "gradient:" << std::endl << gradient << std::endl;
      //  std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;
        //std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;
        //std::cout << "upperBound:" << std::endl << upperBound << std::endl;//直接cout b会rviz卡死

 
        // instantiate the solver
        OsqpEigen::Solver solver;
        // settings
        //solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        // set the initial data of the QP solver
        //矩阵A为m*n矩阵
        solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
        solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
        if(!solver.data()->setGradient(gradient)) return 2; //设置q or f矩阵。当没有时设置为全0向量
       // std::cout << "22222222:" << std::endl ; 
        
        if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
        //std::cout << "1111111:" << std::endl ; 
        
        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 3;//设置线性约束的A矩阵
      // std::cout << "333333:" << std::endl ; 
        if(!solver.data()->setLowerBound(lowerBound)) return 4;//设置下边界
       //std::cout << "4444444:" << std::endl ; 
        if(!solver.data()->setUpperBound(upperBound)) return 5;//设置上边界
      // std::cout << "55555:" << std::endl ; 

        // instantiate the solver
        if(!solver.initSolver()) return 6;
        std::cout << "6666666:" << std::endl ; 
        Eigen::VectorXd QPSolution;

        // solve the QP problem
        if(!solver.solve()) return 7;
        std::cout << "777777:" << std::endl ; 

        // get the controller input
        QPSolution = solver.getSolution();

        //std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;
        std::cout << "=======Radius=================  "<< (QPSolution-path_pt).norm()<< std::endl;
        //std::cout << "=======Radius=================  " << 1<<std::endl;
        sp_radius.push_back((QPSolution-path_pt).norm());
        center_pt.push_back(path_pt);
        return 0;    

        }
        void cloud2Callback(const sensor_msgs::PointCloud2 &msg)
        {
            sensor_msgs::PointCloud out_cloud;
            sensor_msgs::convertPointCloud2ToPointCloud(msg, out_cloud);
            vec_obs_ = DecompROS::cloud_to_vec(out_cloud);
           // ROS_INFO("cloud ok");
        }
        void cmdHandleCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
        {
        point_now(0) = msg->position.x;
        point_now(1) = msg->position.y;
        point_now(2) = msg->position.z;
        //ROS_INFO("p1 ok");
        }
        void cmd_predictHandleCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
        {
        point_pred(0) = msg->position.x;
        point_pred(1) = msg->position.y;
        point_pred(2) = msg->position.z;
        //ROS_INFO("p2 ok");
        }
        void sfc_along_pathCallback(const ros::TimerEvent &e)//定时启动  
        {
            
        
            vec_Vec3f sfc_path;
            int index = poly_constraints.size();
            double dis;
            dis = (point_now-point_pred).norm();
            //如果是出发和到达 那么不用规划
            if(dis<0.05)
            {
               // ROS_INFO("uuuuuu = %lf",dis);
                return;
                
               // ROS_INFO("cloud ok");
            }
            

            if (index >= 1)//如果有已经存在的多面体 先检查是否走出这个范围
            {
            //directly check the reference ellipsoid shape within the polytope
            auto temp_A = (poly_constraints.back()).A();
            auto temp_b = (poly_constraints.back()).b();
            //bool flag = true;

            LinearConstraint3D cs(temp_A,temp_b); 
                // with little inflation
                if (! cs.inside(point_now))//1 在外部 已经要走出范围继续
                {
                    //flag = false;
                    //再规划一段走廊
                    sfc_path.push_back(point_now);
                    sfc_path.push_back(point_pred);
                    EllipsoidDecomp3D decomp_util;
                    decomp_util.set_obs(vec_obs_);
                    decomp_util.set_local_bbox(Vec3f(1, 1, 1));
                    decomp_util.dilate(sfc_path);
                    auto path_poly = decomp_util.get_polyhedrons();
                    vec_E<LinearConstraint3D> css = decomp_util.get_constraints();
                    polys.push_back(path_poly[0]);//保存多面体
                    poly_constraints.push_back(css[0]);//保存约束
                    display_sfc();
                   

                    //std::cout<<"=============== A =="<<std::endl<<css[0].A()<<std::endl;
                    //std::cout<<"=============== b =="<<std::endl<<css[0].b()<<std::endl;
                   // int sta = Radius_Cal((point_now+point_pred)/2,css[0].A(),css[0].b());
                   
                   //   center_point= (point_now+point_pred)/2;  
                   // Sphere_Cal((point_now+point_pred)/2,css[0].A(),css[0].b());
                    
                    //std::cout<<"=======center=========="<<((point_now+point_pred)/2)<<"============="<<std::endl;
                } 
            }
            else //初始状态或者终止状态
            {
                    sfc_path.push_back(point_now);
                    sfc_path.push_back(point_pred);
                    EllipsoidDecomp3D decomp_util;
                    decomp_util.set_obs(vec_obs_);
                    decomp_util.set_local_bbox(Vec3f(1, 1, 0.5));
                    decomp_util.dilate(sfc_path);
                    auto path_poly = decomp_util.get_polyhedrons();
                    vec_E<LinearConstraint3D> css = decomp_util.get_constraints();
                    polys.push_back(path_poly[0]);//保存多面体
                    poly_constraints.push_back(css[0]);//
                    display_sfc();
                   // std::cout<<"=============== A =="<<std::endl<<css[0].A()<<std::endl;
                   // std::cout<<"=============== b =="<<std::endl<<css[0].b()<<std::endl;
                    
                   // Sphere_Cal((point_now+point_pred)/2,css[0].A(),css[0].b());
                    //int sta = Radius_Cal((point_now+point_pred)/2,css[0].A(),css[0].b());
                    //center_point= (point_now+point_pred)/2;
                    
                   // std::cout<<"================="<<(point_now+point_pred)/2<<"============="<<std::endl;
                    //            if(sta>0)//提前终止了
                    //{
                      //      sp_radius.push_back(0.2);
                    //        center_pt.push_back((point_now+point_pred)/2);
                    //}


  
            }

        
    }
    void display_sfc()
    {
        if(poly_constraints.size()<1)
            {
               // ROS_INFO("No poly");
                return;
            }
                decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
                poly_msg.header.frame_id = "world";
                poly_pub.publish(poly_msg);

    }
        void sfc_visualCallback(const ros::TimerEvent &e)
    {
        
            if(poly_constraints.size()<1)
            {
               // ROS_INFO("No poly");
                return;
            }
               // decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
               // poly_msg.header.frame_id = "world";
               // poly_pub.publish(poly_msg);

                /*
                double gap_width = Sphere_Cal(point_now,(poly_constraints.back()).A(),(poly_constraints.back()).b());
              //  std::cout<<"Gap == "<<gap_width<<std::endl;
                std_msgs::Float64 roll ;
                 if (abs(gap_width)<0.1)
    
                 {
                   // std::cout<<"Too Narrow"<<std::endl;
                    return;
                     code 
                 }
                 else if (abs(gap_width)<1.4) 
                 {
                    roll.data = acos(abs(gap_width)/1.4);
                    //std::cout<<"TILT  "<<abs(gap_width)<<std::endl;
                 
                 }
                 else{
                    std::cout<<"LEVEL  "<<abs(gap_width)<<std::endl;
                    roll.data = 0.0;
                 }
                 roll_pub.publish(roll);

                 */

        return;
    }

    

};




int main(int argc, char ** argv){
  ros::init(argc, argv, "sfc_gen_node");
  ros::NodeHandle nh("~");

  
 SFC_GEN sfc_generation(nh);
  
 // test();
 ros::Duration(1.0).sleep();
 ros::spin();

  return 0;
}
