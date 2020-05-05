#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/
Eigen::MatrixXd TrajectoryGeneratorWaypoint::getQ(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    // calculate Matrix Q_k of the seg_index-th segment
    MatrixXd Q_k = MatrixXd::Zero(p_num1d, p_num1d);
    for (int i = 0; i < p_num1d; i++)
    {
        for (int j = 0; j < p_num1d; j++)
        {
            if (i >= p_num1d - d_order && j >= p_num1d - d_order)
            {
                Q_k(i, j) = (Factorial(i) / Factorial(i - d_order)) * ((Factorial(j) / Factorial(j - d_order))) /
                            (i + j - 2 * d_order + 1) * pow(Time(seg_index), (i + j - 2 * d_order + 1)); // Q of one segment
            }
        }
    }
   //get Q for calculation of cost
    return Q_k;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getM(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    MatrixXd M_k = MatrixXd::Zero(p_num1d, p_num1d);
    VectorXd t_pow = VectorXd::Zero(p_num1d);
    for(int i = 0; i < p_num1d; i++)
    {
        t_pow(i) = pow(Time(seg_index),i);
    }
    //Because we can choose if use mini-snap or mini-jerk in launch file,so here we do it both.
    if(p_num1d == 6)        // minimum jerk
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3);
    }
    else if(p_num1d == 8)   // minimum snap
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     0     ,     6     ,      0     ,      0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),    t_pow(6),    t_pow(7),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),  6*t_pow(5),  7*t_pow(6),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3), 30*t_pow(4), 42*t_pow(5),
               0,     0   ,     0     ,     6     , 24*t_pow(1), 60*t_pow(2),120*t_pow(3),210*t_pow(4);
    }
    
    return M_k;
}
Eigen::MatrixXd TrajectoryGeneratorWaypoint::getCt(const int seg_num, const int d_order)
{
    int d_num = 2 * d_order * seg_num;
    int df_and_dp_num = d_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * d_order + mid_waypts_num;//constraint 约束项
    Eigen::MatrixXd Ct = MatrixXd::Zero(d_num, df_and_dp_num);    
    //initialize
    Ct.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);
    Ct.block(d_num - d_order, df_num - d_order, d_order, d_order) = MatrixXd::Identity(d_order, d_order);
    for(int mid_waypts_index = 0; mid_waypts_index < mid_waypts_num; mid_waypts_index++)
    {
        //位置为固定项
        Ct(d_order+2*d_order*mid_waypts_index, d_order+mid_waypts_index) = 1;
        Ct(d_order+(d_order+2*d_order*mid_waypts_index), d_order+mid_waypts_index) = 1;
        
        //速度，自由项
        Ct(d_order+1+2*d_order*mid_waypts_index, df_num+(d_order-1)*mid_waypts_index) = 1;
        Ct(d_order+(d_order+1+2*d_order*mid_waypts_index), df_num+(d_order-1)*mid_waypts_index) = 1;

        ////速度，自由项
        Ct(d_order+2+2*d_order*mid_waypts_index, (df_num+1)+(d_order-1)*mid_waypts_index) = 1;
        Ct(d_order+(d_order+2+2*d_order*mid_waypts_index), (df_num+1)+(d_order-1)*mid_waypts_index) = 1;

        if(d_order == 4)  // minimum snap
        {
            ////jerk，自由项
            Ct(d_order+3+2*d_order*mid_waypts_index, (df_num+2)+(d_order-1)*mid_waypts_index) = 1;
            Ct(d_order+(d_order+3+2*d_order*mid_waypts_index), (df_num+2)+(d_order-1)*mid_waypts_index) = 1;   
        }
    }

    return Ct;
}

Eigen::VectorXd TrajectoryGeneratorWaypoint::QP_optimization(const Eigen::MatrixXd &Q,
                                                                  const Eigen::MatrixXd &M,
                                                                  const Eigen::MatrixXd &Ct,
                                                                  const Eigen::VectorXd &waypoint,
                                                                  const Eigen::VectorXd &beginstate,
                                                                  const Eigen::VectorXd &endstate,
                                                                  const int seg_num,
                                                                  const int d_order)
{
    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    int df_and_dp_num = d_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * d_order + mid_waypts_num;
    int dp_num = (d_order - 1) * mid_waypts_num;

    Eigen::MatrixXd C = Ct.transpose();
    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::MatrixXd R = C * M_inv_tran * Q * M_inv * Ct;
    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);

    // compute dF
    Eigen::VectorXd dF(df_num);
    dF.head(d_order) = beginstate;    // begin state: pos,vel,acc,(jerk)
    
    dF.segment(d_order, mid_waypts_num) = waypoint.segment(1,waypoint.rows()-2);  // middle waypoints: pos
    dF.tail(d_order) = endstate;      // end state: pos,vel,acc,jerk
    Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;   // closed-form solution of Unconstrained quadratic programming
    Eigen::VectorXd dF_and_dP(df_and_dp_num);
    dF_and_dP << dF, dP;
    Eigen::VectorXd PolyCoeff = M_inv * Ct * dF_and_dP;   // all coefficients of one segment

    return PolyCoeff;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    const int p_order   = 2 * d_order - 1;              // the order of polynomial
    const int p_num1d   = p_order + 1;                  // the number of variables in each segment
    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);
    _Q = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    _M = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    _Ct = MatrixXd::Zero(2 * d_order * m, d_order * (m + 1));

    /*   Produce Mapping Matrix M to the entire trajectory, M is a mapping matrix that maps polynomial coefficients to derivatives.   
        And we must get the cost matrix Q*/
    for(int i = 0; i < m; i++)
    {
        // calculate Matrix Q
        _Q.block(i*p_num1d, i*p_num1d, p_num1d, p_num1d) = getQ(p_num1d, d_order, Time,i);
        // calculate Matrix M
        _M.block(i*p_num1d, i*p_num1d, p_num1d, p_num1d) = getM(p_num1d, d_order, Time, i);
    }
    // calculate Matrix Ct
    _Ct = getCt(m, d_order);

    /*  Produce the dereivatives in X, Y and Z axis directly.  */
    MatrixXd StartState = MatrixXd::Zero(d_order, 3);
    MatrixXd EndState = MatrixXd::Zero(d_order, 3);
    StartState.row(0) = Path.row(0);
    StartState.row(1) = Vel.row(0);
    StartState.row(2) = Acc.row(0);
    EndState.row(0) = Path.row((Path.rows()-1));
    EndState.row(1) = Vel.row(1);
    EndState.row(2) = Acc.row(1);
    if(d_order == 4)
    {
        StartState.row(3) = VectorXd::Zero(3);  // mini-snap,need jerk continuty
        EndState.row(3) = VectorXd::Zero(3); 
    }
    //cout<<"startstate: "<<StartState<<endl;
    Px = QP_optimization(_Q, _M, _Ct, Path.col(0), StartState.col(0), EndState.col(0), m, d_order);
    Py = QP_optimization(_Q, _M, _Ct, Path.col(1), StartState.col(1), EndState.col(1), m, d_order);
    Pz = QP_optimization(_Q, _M, _Ct, Path.col(2), StartState.col(2), EndState.col(2), m, d_order);
    for(int i = 0; i < m; i++)
    {
        PolyCoeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(2*p_num1d, p_num1d) = Pz.segment(p_num1d*i, p_num1d);
    }
    return PolyCoeff;
}
