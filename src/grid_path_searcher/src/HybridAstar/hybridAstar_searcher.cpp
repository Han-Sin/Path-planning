#include "hybridAstar_searcher.h"
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
using namespace std;
using namespace Eigen;

double hybridAstar_searcher::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                                           double& optimal_time) {
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d  = t_bar;

  for (auto t : ts) {
    if (t < t_bar) continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost) {
      cost = c;
      t_d  = t;
    }
  }
  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}
bool hybridAstar_searcher::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                       double time_to_goal) {
  /* ---------- get coefficient ---------- */
  const Vector3d p0  = state1.head(3);
  const Vector3d dp  = state2.head(3) - p0;
  const Vector3d v0  = state1.segment(3, 3);
  const Vector3d v1  = state2.segment(3, 3);
  const Vector3d dv  = v1 - v0;
  double         t_d = time_to_goal;
  MatrixXd       coef(3, 4);
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
  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++) t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++) {
      poly1d     = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim)   = (Tm * poly1d).dot(t);
      acc(dim)   = (Tm * Tm * poly1d).dot(t);

      if (fabs(acc(dim)) > max_acc_) {
        return false;
      }
    }

    if (coord(0) < gl_xl || coord(0) >= gl_xu || coord(1) < gl_yl ||
        coord(1) >= gl_yu || coord(2) < gl_zl || coord(2) >= gl_zu) {
      return false;
    }

    if (!isFree(coord2gridIndex(coord))) {
      return false;
    }
  }
  coef_shot_    = coef;
  t_shot_       = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> hybridAstar_searcher::cubic(double a, double b, double c, double d) {
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> hybridAstar_searcher::quartic(double a, double b, double c, double d, double e) {
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double         y1 = ys.front();
  double         r  = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}
void hybridAstar_searcher::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                                    double tau) {
  for (int i = 0; i < 3; ++i) phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

// namespace fast_planner

int hybridAstar_searcher::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
           Eigen::Vector3d end_pt, Eigen::Vector3d end_v,ros::Publisher &  path_pub){
    //ROS_INFO_STREAM("start!!!"<<start_pt);
    ros::Time time_1 = ros::Time::now();
    //index of start_point and end_point
    start_acc_ = start_a;
    start_vel_ = start_v;
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;
    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    is_shot_succ_ = false;
    // currentPtr represents the node with lowest f(n) in the open_list
    //put start node in open set
    double time_to_goal;
    Eigen::VectorXd end_state(6),start_state(6);
    end_state.head(3) = end_pt;
    end_state.tail(3) = end_v;
    start_state.head(3) = start_pt;
    start_state.tail(3) = start_v;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->cameFrom = NULL;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->state.head(3) = start_pt;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->state.tail(3) = start_v;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->gScore = 0;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->fScore = lambda_heu_*estimateHeuristic(start_state,end_state,time_to_goal);
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->node_state = IN_OPEN_SET;
    openSet.insert( make_pair(GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->fScore, GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]));
    use_node_num_ +=1;
    //GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code beow
    *
    *
    remove from openlist, and add to close list.
    */
    bool init_search = true;
    // this is the main loop
    int num = 0;
    while ( !openSet.empty() ){
        //ROS_WARN("3333333333333333333");
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below

        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        GridNodePtr currentPtr;
        std::multimap<double, GridNodePtr> ::iterator it;
        it = openSet.begin();
        currentPtr = (*it).second;
        openSet.erase(it);
        currentPtr->node_state = IN_CLOSE_SET;
        iter_num_+=1;



        visualization_msgs::Marker _traj_vis;
        _traj_vis.header.stamp       = ros::Time::now();
        _traj_vis.header.frame_id    = "world";
        _traj_vis.ns = "traj_node/trajectory_waypoints";
        _traj_vis.id = num;
        num++;
        //ROS_INFO_STREAM("NUM!!!: "<<num);
        _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        _traj_vis.action = visualization_msgs::Marker::ADD;
        _traj_vis.scale.x = 0.05;
        _traj_vis.scale.y = 0.05;
        _traj_vis.scale.z = 0.05;
        _traj_vis.pose.orientation.x = 0.0;
        _traj_vis.pose.orientation.y = 0.0;
        _traj_vis.pose.orientation.z = 0.0;
        _traj_vis.pose.orientation.w = 1.0;
        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 0.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 1.0;//红色线
        _traj_vis.points.clear();
        Vector3d pos;
        geometry_msgs::Point pt;
        pos  = currentPtr->state.head(3);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _traj_vis.points.push_back(pt);
        path_pub.publish(_traj_vis);








        //double thres = -0.18*(currentPtr->state.head(3)-end_pt).norm()+1;
        //double   p = (rand()%10000)/10000.0;
        //currentPtr->index == goalIdx
        /*if( (currentPtr->state.head(3)-end_pt).norm()<1) {
            has_path_ = true;
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[hybrid A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
            return 1;
        }*/
        /*if( currentPtr->index == goalIdx){
            has_path_ = true;
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            estimateHeuristic(currentPtr->state,end_state,time_to_goal);
            computeShotTraj(currentPtr->state,end_state,time_to_goal);
            if (terminatePtr->cameFrom == NULL && !is_shot_succ_)
            { ROS_WARN("hybird Astar dead!!!,NO PATH!!!");
              return 0;}
            else{
              ROS_WARN("[hybrid A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
              return 1;
            }
        }*/
        //ROS_WARN("222222222222");
        
        if((currentPtr->state.head(3)-end_pt).norm()<2){
          estimateHeuristic(currentPtr->state,end_state,time_to_goal);
          computeShotTraj(currentPtr->state,end_state,time_to_goal);
          if (is_shot_succ_)
          { 
            has_path_ = true;
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[hybrid A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
            //ROS_ERROR("")
            return 1;
          }
          else{
            ROS_WARN("SHOT DEAD!!!!!");
          }
        }

        //ROS_WARN("111111111111");
        //is_shot_succ_ = false;
        //ROS_INFO_STREAM("THRES:  "<<thres);
        /*if(p<thres){
          ROS_INFO_STREAM("THRES:  "<<thres);
          estimateHeuristic(currentPtr->state,end_state,time_to_goal);
          computeShotTraj(currentPtr->state,end_state,time_to_goal);
          if (is_shot_succ_)
          { 
            has_path_ = true;
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[hybrid A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
            return 1;
          }
        }*/
        /*
        estimateHeuristic(currentPtr->state,end_state,time_to_goal);
        computeShotTraj(currentPtr->state,end_state,time_to_goal);
        if (is_shot_succ_)
        { 
          has_path_ = true;
          ros::Time time_2 = ros::Time::now();
          terminatePtr = currentPtr;
          ROS_WARN("[hybrid A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
          return 1;
        }*/
        //get the succetion
        //AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself
        //propagate
        Eigen::Matrix<double, 6, 1> cur_state = currentPtr->state;
        Eigen::Matrix<double, 6, 1> pro_state;
        Eigen::Vector3d             um;
        vector<Eigen::Vector3d> inputs;
        vector<double>          durations;
        double res = 1 / 2.0, time_res = 1 / 4.0, time_res_init = 1 / 8.0;
        //max_tau 0.25 init max_tau 0.8 acc 0.3 res = 0.5
        
        if (init_search) {
          inputs.push_back(start_acc_);
          for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_;
            tau += time_res_init * init_max_tau_)
            durations.push_back(tau);
            } else {
          for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
              for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
                for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res) {
                  um << ax, ay, az;
                  inputs.push_back(um);}
          for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
            durations.push_back(tau);}
        init_search = false;
        //begin to propagate
        for(int i= 0;i<inputs.size();i++){
          for(int j=0;j<durations.size();j++){
            init_search = false;
            um = inputs[i];
            double tau = durations[j];
            stateTransit(cur_state,pro_state,um,tau);
            //ROS_INFO_STREAM("cur->pro: "<<pro_state);
            //障碍检测
            if(!isFree(coord2gridIndex(pro_state.head(3)))) continue;
            Eigen::Matrix<double,6,1> xt;
            Eigen::Vector3d             pos;
            bool is_occ = false;
            for (int k = 1; k <= check_num_; ++k) {
              double dt = tau * double(k) / double(check_num_);
              stateTransit(cur_state, xt, um, dt);
              pos = xt.head(3);
              if(!isFree(coord2gridIndex(pos))){
                is_occ = true;
                break;
              }
            }
            if(is_occ)             continue;

            Eigen::Vector3i pro_index = coord2gridIndex(pro_state.head(3));
            //close list 
            //vel feasiable
            Eigen::Vector3d pro_v = pro_state.tail(3);
            if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_) {
              // cout << "vel infeasible" << endl;
              continue;
            }
            //相对于之前是不是发生过位移，如果没有则舍弃
            Eigen::Vector3i diff      = pro_index - currentPtr->index;
            /*if (diff.norm() == 0 && || diff_time == 0)) {
              continue;
            }*/
            if (diff.norm() == 0) {
              continue;
            }
            double tmp_g_score,tmp_f_score,tmp_h_score,time_to_goal;
            tmp_h_score = lambda_heu_*estimateHeuristic(pro_state,end_state,time_to_goal);
            tmp_g_score = (um.squaredNorm()+w_time_)*tau+currentPtr->gScore;
            tmp_f_score = tmp_h_score+tmp_g_score;
            //ROS_INFO_STREAM("LAMBDA"<<lambda_heu_);
            if(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->node_state==IN_CLOSE_SET) {
              /*if(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore>tmp_f_score){
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->cameFrom = currentPtr;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->state = pro_state;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->gScore = tmp_g_score;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore = tmp_f_score;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->node_state = IN_OPEN_SET;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->duration = tau;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->input = um;
                openSet.insert( make_pair(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore, GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]));
                use_node_num_ +=1;
              }*/
              //ROS_WARN("closelist!!!!!");
              //ROS_INFO_STREAM("cur_state: "<<cur_state.head(3)<<"  pro_state: "<<pro_state);
              continue;
              
              
            }
            else if(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->node_state==IN_OPEN_SET){
              if(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore>tmp_f_score){
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->cameFrom = currentPtr;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->state = pro_state;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->gScore = tmp_g_score;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore = tmp_f_score;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->node_state = IN_OPEN_SET;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->duration = tau;
                GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->input = um;
                //ROS_WARN("222222222222222");
                //for()
                //openSet.insert( make_pair(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore, GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]));
                use_node_num_ +=1;
              }
            }
            else if(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->node_state==NOT_EXPAND)
            { 
              //ROS_WARN("666666666666666");
              GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->cameFrom = currentPtr;
              GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->state = pro_state;
              GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->gScore = tmp_g_score;
              GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore = tmp_f_score;
              GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->node_state = IN_OPEN_SET;
              GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->duration = tau;
              GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->input = um;
              openSet.insert( make_pair(GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]->fScore, GridNodeMap[pro_index[0]][pro_index[1]][pro_index[2]]));
              use_node_num_ +=1;
            }
            else{
              //ROS_WARN("There must be error!");
            }
          }
        }
        //ROS_WARN("55555555555");
        //if search fails
      }
    //ROS_WARN("666666666666");
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Hybrid-Astar path finding is %f", (time_2 - time_1).toSec() ); 
}

vector<Eigen::Vector3d> hybridAstar_searcher::getKinoTraj(double delta_t){
  GridNodePtr node = terminatePtr;
  /* ---------- get traj of searching ---------- */
  vector<Vector3d> state_list;
  Matrix<double, 6, 1> x0, xt;
  while (node->cameFrom != NULL) {
    Vector3d ut       = node->input;
    double   duration = node->duration;
    x0                = node->cameFrom->state;

    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->cameFrom;
  }
  reverse(state_list.begin(), state_list.end());
  if(is_shot_succ_){
    //ROS_INFO_STREAM("!!!!");
    Vector3d coord;
    VectorXd poly1d, time(4);
    double t;
    for ( t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 4; j++) time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
    if(t!=t_shot_){
      t = t_shot_;
      for (int j = 0; j < 4; j++) time(j) = pow(t, j);
      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }
  return state_list;//得到轨迹点的坐标
}

vector<Eigen::Vector3d> hybridAstar_searcher::getKinoVel(double delta_t){
  GridNodePtr node = terminatePtr;
  /* ---------- get traj of searching ---------- */
  vector<Vector3d> state_list;
  Matrix<double, 6, 1> x0, xt;
  while (node->cameFrom != NULL) {
    Vector3d ut       = node->input;
    double   duration = node->duration;
    x0                = node->cameFrom->state;
    //ROS_INFO_STREAM("duration: "<<duration);
    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.tail(3));
    }
    node = node->cameFrom;
  }
  reverse(state_list.begin(), state_list.end());
  if(is_shot_succ_){
    Vector3d Vel_xyz;
    VectorXd poly1d, time(4);
    double t;
    for ( t = delta_t; t <= t_shot_; t += delta_t) {
      time(0) = 0;
      for (int j = 1; j < 4; j++) time(j) = j*pow(t, j-1);
      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        Vel_xyz(dim) = poly1d.dot(time);
        ROS_INFO_STREAM("vel: "<<Vel_xyz);
      }
      state_list.push_back(Vel_xyz);
    }
    if(t!=t_shot_){
      t = t_shot_;
      time(0) = 0;
      for (int j = 1; j < 4; j++) time(j) = j*pow(t, j-1);
      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        Vel_xyz(dim) = poly1d.dot(time);
      }
      ROS_INFO_STREAM("vel: "<<Vel_xyz);
      state_list.push_back(Vel_xyz);
    } 
  }
  return state_list;

}





void hybridAstar_searcher::setParam(ros::NodeHandle& nh) {
  nh.param("search/max_tau", max_tau_, -1.0);//0.6
  nh.param("search/init_max_tau", init_max_tau_, -1.0);//0.8
  nh.param("search/max_vel", max_vel_, -1.0);//5
  nh.param("search/max_acc", max_acc_, -1.0);//5
  nh.param("search/w_time", w_time_, -1.0);//10
  nh.param("search/lambda_heu", lambda_heu_, -1.0);//5
  nh.param("search/check_num", check_num_, -1);//5
}
