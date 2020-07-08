#include "Astar_searcher.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <utility>
#include "backward.hpp"
#include "GridNode.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <Eigen/Eigen>
#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>

class hybridAstar_searcher:public AstarPathFinder
{
    private:
    int use_node_num_, iter_num_;
    Eigen::Vector3d start_vel_, end_vel_, start_acc_;
    Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
    bool is_shot_succ_ = false;
    Eigen::MatrixXd coef_shot_;
    double t_shot_;
    bool has_path_ = false;
    double max_tau_ = 0.25;
    double init_max_tau_ = 0.8;
    double max_vel_ = 3.0;
    double max_acc_ = 3.0;
    double w_time_ = 10.0;//时间占的权重
    double lambda_heu_ = 10;
    int check_num_ = 5;
    double tie_breaker_ = 1.0 + 1.0 / 10000;
    /*
     nh.param("search/max_tau", max_tau_, -1.0);//0.6
  nh.param("search/init_max_tau", init_max_tau_, -1.0);//0.8
  nh.param("search/max_vel", max_vel_, -1.0);//5
  nh.param("search/max_acc", max_acc_, -1.0);//5
  nh.param("search/w_time", w_time_, -1.0);//10
  nh.param("search/lambda_heu", lambda_heu_, -1.0);//5
  nh.param("search/check_num", check_num_, -1);//5
    */
    public:
        hybridAstar_searcher(){
            phi_          = Eigen::MatrixXd::Identity(6, 6);
            use_node_num_ = 0;
            iter_num_     = 0;
            srand((unsigned)time(NULL));
        };
        ~hybridAstar_searcher(){};
        /* shot trajectory */
        vector<double> cubic(double a, double b, double c, double d);
        vector<double> quartic(double a, double b, double c, double d, double e);
        bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
        vector<Eigen::Vector3d> getKinoTraj(double delta_t);
        vector<Eigen::Vector3d> getKinoVel(double delta_t);
        double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
        /* state propagation */
        void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);
        enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3 }; 
        /*API*/
        void setParam(ros::NodeHandle& nh);
        int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_v,ros::Publisher & path_pub);
        void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,vector<Eigen::Vector3d>& start_end_derivatives);
};


