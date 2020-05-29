#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
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
void TrajectoryGeneratorWaypoint::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);
    // ROS_INFO("gl_xl_init=%f",gl_xl);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}
void TrajectoryGeneratorWaypoint::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = int( (coord_x - gl_xl) * inv_resolution);
    // ROS_INFO("ORI_X=%f     idx_x=%d   ",( (coord_x - gl_xl) * inv_resolution),idx_x);
    int idx_y = int( (coord_y - gl_yl) * inv_resolution);
    int idx_z = int( (coord_z - gl_zl) * inv_resolution);

    double expand_ratio=0;

    double default_resolution=0.2;
    // ROS_INFO("resolution=%f",resolution);
    // int expand_size=(int)(expand_ratio*(double)(default_resolution/resolution));//膨胀栅格数，0时不膨胀，1够用

    int expand_size=(int)(default_resolution/resolution)-1+(int)expand_ratio;

    // if(expand_size<=0)
    //     expand_size=0;
    
    // expand_size=expand_size-1;
    // ROS_INFO("traj   expand_size= %d  ",expand_size);
    

    for (int i=-expand_size;i<=expand_size;i++)
        for (int j=-expand_size;j<=expand_size;j++)
            for (int k=-expand_size;k<=expand_size;k++)
            {
                int temp_x=idx_x+i;
                int temp_y=idx_y+j;
                int temp_z=idx_z+k;

                double rev_x=(double)temp_x/inv_resolution+gl_xl;
                double rev_y=(double)temp_y/inv_resolution+gl_yl;
                double rev_z=(double)temp_z/inv_resolution+gl_zl;

                if( rev_x < gl_xl  || rev_y < gl_yl  || rev_z <  gl_zl ||
                    rev_x >= gl_xu || rev_y >= gl_yu || rev_z >= gl_zu )
                    continue;
//                ROS_WARN("expand suc,%d  %d  %d  ",i,j,k);
                data[temp_x * GLYZ_SIZE + temp_y * GLZ_SIZE + temp_z] = 1;//index(grid)
            }
}

Vector3d TrajectoryGeneratorWaypoint::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;
    return pt;
}
Vector3i TrajectoryGeneratorWaypoint::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
    // ROS_INFO("%f   %f   %f   ->   %d   %d   %d",pt(0),pt(1),pt(2),idx[0],idx[1],idx[2]);
    // ROS_INFO("gl_xl=%f   inv_resolution=%f    ",gl_xl,inv_resolution);
    return idx;
}
Eigen::Vector3d TrajectoryGeneratorWaypoint::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool TrajectoryGeneratorWaypoint::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool TrajectoryGeneratorWaypoint::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool TrajectoryGeneratorWaypoint::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool TrajectoryGeneratorWaypoint::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}
Vector3d TrajectoryGeneratorWaypoint::getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;
    int _poly_num1D = (int)polyCoeff.cols()/3;
    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}
int TrajectoryGeneratorWaypoint::safeCheck( MatrixXd polyCoeff, VectorXd time)
{
    Vector3d pt;
    int unsafe_segment = -1; //-1 -> the whole trajectory is safe

    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01)
        {
          pt = getPosPoly(polyCoeff, i, t);
          Vector3i idx;
          idx = coord2gridIndex(pt);
          if(isOccupied(idx)){
            unsafe_segment = i;
            return unsafe_segment;
          }
        }
    }
    return unsafe_segment;
}




bool FlightCorridor::check_cube_safe(FlightCube cube)
{
//   return isOccupied(1,1,1);
  for (int i=cube.start_node->index[0]-cube.x_neg_int;i<=cube.start_node->index[0]+cube.x_pos_int;i++)
    for (int j=cube.start_node->index[1]-cube.y_neg_int;j<=cube.start_node->index[1]+cube.y_pos_int;j++)
      for (int k=cube.start_node->index[2]-cube.z_neg_int;k<=cube.start_node->index[2]+cube.z_pos_int;k++)
      {
        if (isOccupied(i,j,k))
        {
            // ROS_INFO("collision with  %d  %d  %d     start_node_x=%d",i,j,k,cube.start_node->index[0]);
            return 0;
        }
      }
  return 1;
}


void FlightCorridor::set_safe_force(FlightCube &cube)
{
    //   return isOccupied(1,1,1);
  for (int i=cube.start_node->index[0]-cube.x_neg_int;i<=cube.start_node->index[0]+cube.x_pos_int;i++)
    for (int j=cube.start_node->index[1]-cube.y_neg_int;j<=cube.start_node->index[1]+cube.y_pos_int;j++)
      for (int k=cube.start_node->index[2]-cube.z_neg_int;k<=cube.start_node->index[2]+cube.z_pos_int;k++)
      {
          if (isOccupied(i,j,k))
          {
            data[i * GLYZ_SIZE + j * GLZ_SIZE + k]=0;
            ROS_INFO("setting %d %d %d safe!!!",i,j,k);
          }
      }
}

FlightCube FlightCorridor::expand_cube(FlightCube &cube)
{
    int max_expand_size=4;
    int x_pos_origin=cube.x_pos_int;
    int x_neg_origin=cube.x_neg_int;
    int y_pos_origin=cube.y_pos_int;
    int y_neg_origin=cube.y_neg_int;
    int z_pos_origin=cube.z_pos_int;
    int z_neg_origin=cube.z_neg_int;


    bool at_least_one_suc_flag=1;
    while(at_least_one_suc_flag)
    {
        at_least_one_suc_flag=0;
        //for x
        if(cube.x_pos_int-x_pos_origin<=max_expand_size)
        {
            cube.x_pos_int++;
            if(!check_cube_safe(cube))
                cube.x_pos_int--;
            else
                at_least_one_suc_flag=1;
        }

        if(cube.x_neg_int-x_neg_origin<=max_expand_size)
        {
            cube.x_neg_int++;
            if(!check_cube_safe(cube))
                cube.x_neg_int--;
            else
                at_least_one_suc_flag=1;
        }

        
        // for y
        if(cube.y_pos_int-y_pos_origin<=max_expand_size)
        {
            cube.y_pos_int++;
            if(!check_cube_safe(cube))
                cube.y_pos_int--;
            else
                at_least_one_suc_flag=1;
        }
        

        if(cube.y_neg_int-y_neg_origin<=max_expand_size)
        {
            cube.y_neg_int++;
            if(!check_cube_safe(cube))
                cube.y_neg_int--;
            else
                at_least_one_suc_flag=1;  
        }

        
        // for z
        if(cube.z_pos_int-z_pos_origin<=max_expand_size)
        {
            cube.z_pos_int++;
            if(!check_cube_safe(cube))
                cube.z_pos_int--;
            else
                at_least_one_suc_flag=1;
        }
        


        if(cube.z_neg_int-z_neg_origin<=max_expand_size&&cube.start_node->index[2]-cube.z_neg_int-1>=0)
        {   
            cube.z_neg_int++;
            if(!check_cube_safe(cube))
                cube.z_neg_int--;
            else
                at_least_one_suc_flag=1;
        }
        
        // ROS_INFO("x= %d  %d  y=%d  %d   z=%d  %d ",cube.x_pos_int,cube.x_neg_int,cube.y_pos_int,
        // cube.y_neg_int,cube.z_pos_int,cube.z_neg_int);
    }
    return cube;
    // for(int i=-max_expand_size;i<=max_expand_size;i++)
    //     for(int j=-max_expand_size;j<=max_expand_size;j++)
    //         for(int k=-max_expand_size;k<=max_expand_size;k++)
    //         {
    //             if(i>0)
    //             {
    //                 cube.x_pos_int+=i;
    //             }
    //             else
    //             {
    //                 cube.x_neg_int+=i;
    //             }
                
    //             if(i>0)
    //             {
    //                 cube.x_pos_int+=i;
    //             }
    //             else
    //             {
    //                 cube.x_neg_int+=i;
    //             }
    //         }
}

void FlightCorridor::update_attributes(FlightCube &cube)
{
    {
        Vector3i temp_idx(cube.start_node->index[0]-cube.x_neg_int,cube.start_node->index[1]-cube.y_neg_int,
        cube.start_node->index[2]-cube.z_neg_int);
        Vector3d temp_coord=gridIndex2coord(temp_idx);
        cube.x_neg=cube.start_node->coord[0]-temp_coord[0]+0.5*resolution;
        cube.y_neg=cube.start_node->coord[1]-temp_coord[1]+0.5*resolution;
        cube.z_neg=cube.start_node->coord[2]-temp_coord[2]+0.5*resolution;
        cube.borders[0]=cube.start_node->coord[0]-cube.x_neg;
        cube.borders[2]=cube.start_node->coord[1]-cube.y_neg;
        cube.borders[4]=cube.start_node->coord[2]-cube.z_neg;
    }
  
    {
        Vector3i temp_idx(cube.start_node->index[0]+cube.x_pos_int,cube.start_node->index[1]+cube.y_pos_int,
        cube.start_node->index[2]+cube.z_pos_int);
        Vector3d temp_coord=gridIndex2coord(temp_idx);
        cube.x_pos=temp_coord[0]-cube.start_node->coord[0]+0.5*resolution;
        cube.y_pos=temp_coord[1]-cube.start_node->coord[1]+0.5*resolution;
        cube.z_pos=temp_coord[2]-cube.start_node->coord[2]+0.5*resolution;
        cube.borders[1]=cube.start_node->coord[0]+cube.x_pos;
        cube.borders[3]=cube.start_node->coord[1]+cube.y_pos;
        cube.borders[5]=cube.start_node->coord[2]+cube.z_pos;
    }
}







//for bezier generate
MatrixXd BezierTrajOptimizer::getQ(const int vars_number, const vector<double> Time, const int seg_index){
    // calculate Matrix Q_k of the seg_index-th segment
    MatrixXd Q_k = MatrixXd::Zero(vars_number, vars_number);
    int d_order = (traj_order+1)/2;//for us, is 4 here 
    for (int i = 0; i < vars_number; i++)
    {
        for (int j = 0; j < vars_number; j++)
        {
            if (i >= vars_number - d_order && j >= vars_number - d_order)
            {
                Q_k(i, j) = (factorial(i) / factorial(i - d_order)) * ((factorial(j) / factorial(j - d_order))) /
                            (i + j - 2 * d_order + 1) * pow(Time[seg_index], (i + j - 2 * d_order + 1)); // Q of one segment
            }
        }
    }
   //get Q for calculation of cost
    return Q_k;
}
Eigen::MatrixXd BezierTrajOptimizer::getM(const int vars_number, const vector<double> Time, const int seg_index){
    MatrixXd M_k = MatrixXd::Zero(vars_number, vars_number);
    VectorXd t_pow = VectorXd::Zero(vars_number);
    for(int i = 0; i < vars_number; i++)
    {
        t_pow(i) = pow(Time[seg_index],i);
    }
    M_k = M;
    for(int i=0;i<vars_number;i++){
        M_k.row(i) = M_k.row(i)/t_pow(i);    
    }
    return M_k;
}    


int BezierTrajOptimizer::bezierCurveGeneration( 
    FlightCorridor corridor,
    const double max_vel, 
    const double max_acc,
    Vector3d start_pos,
    Vector3d end_pos,
    VectorXd time,
    Vector3d start_vel,
    Vector3d start_acc
    )
{   
    //ROS_INFO("zheli!!!!!!");
    segs = corridor.cubes.size();
    ROS_INFO_STREAM("segs: "<<segs);
    vector<double>  time_intervals;
    for(int i=0;i<segs;i++)
        time_intervals.push_back(time(i));


    /*PolyCoeff = MatrixXd::Zero(segs, all_vars_number);
        PolyTime  = VectorXd::Zero(segs);
        obj = 0.0;
        
        int var_shift = 0;

        MatrixXd Q_o(vars_number,vars_number);
        //    int s1d1CtrlP_num = traj_order + 1;
        //    int s1CtrlP_num   = 3 * s1d1CtrlP_num;
        //int min_order_l = floor(minimize_order);
        //int min_order_u = ceil (minimize_order);

        for(int i = 0; i < segs; i++ )
        {   
            PolyTime(i) = time[i];

            for(int j = 0; j < all_vars_number; j++)
                {
                    PolyCoeff(i , j) = d_var[j + var_shift];
                    // cout<<"coeff in is  "<<PolyCoeff(i , j)<<"i="<<i<<"  j="<<j<<endl;;
                }
            var_shift += all_vars_number;     
        }   
    */
    //time_intervals.push_back(1);
    int vars_number = traj_order+1;
    int all_vars_number = 3*vars_number;//XYZ
    int nx = segs*3*vars_number;//需要优化的系数的个数ca
    double c[nx];
    double  xupp[nx];    
    char   ixupp[nx];
    double  xlow[nx];
    char   ixlow[nx];
    for(int i=0;i<nx;i++){
        c[i] = 0.0;
        xlow[i] = 0.0;
        ixlow[i] = 0;
        xupp[i] = 0.0;
        ixupp[i] = 0;
    }
    //c00x c01x ... c07x  c00y c01y...c07z  c00z c01z... c07z
    //c10x
    //等式约束部分
    int equ_con_s_num = 3 * 3; // start state p v a 
    int equ_con_e_num = 3 * 3; // end state p v a 
    int equ_con_continuity_num = 3 * 4 * (segs - 1);
    int equ_con_num = equ_con_s_num + equ_con_e_num + equ_con_continuity_num;  // p, v, a j in x, y, z axis in each segment's joint position
    //int ieq_con_pos_num = 0; // all control points within the polytopes, each face assigned a linear constraint
    double b[equ_con_num];//start p(xyz),v(xyz),a(xyz),j(xyz)->end:p(xyz),v(xyz),a(xyz),j(xyz)->0,0,
    int my = equ_con_num;
    //ROS_INFO("zheli2!!");
    // ROS_INFO_STREAM("equ_con_num"<<equ_con_num);
    // ROS_INFO_STREAM("start"<<start_pos);
    // ROS_INFO_STREAM ("end"<<end_pos);
    for(int i = 0; i < equ_con_num; i ++ )//起点-》终点-》中间点
    { 
        double beq_i;//pvaj
        if(i < 3)                    beq_i = start_pos(i); //p
        else if (i >= 3  && i < 6  ) beq_i = start_vel(i-3);//v 
        else if (i >= 6  && i < 9  ) beq_i = start_acc(i-6);//a
        else if (i >= 9 && i < 12 ) beq_i = end_pos(i-9);//pend_pos(i)
        else if (i >= 12 && i < 15 ) beq_i = 0;//end:v
        else if (i >= 15 && i < 18 ) beq_i = 0;//end:a
        else beq_i = 0.0;//连续性约束
        b[i] = beq_i;
    }
    int nn_idx  = 0;
    int row_idx = 0;
    int nnzA  = (1 * 3 + 2 * 3 + 3 * 3 ) * 2 + (segs - 1) * (2 + 4 + 6 + 8) * 3;


    double dA[nnzA];
    int irowA[nnzA];
    int jcolA[nnzA];
    //等式约束
    // stacking all equality constraints

    //   Start position 
        // position :
        for(int i = 0; i < 3; i++)
        {  // loop for x, y, z       
            dA[nn_idx] = 1.0;
            irowA[nn_idx] = row_idx;
            jcolA[nn_idx] = i * vars_number;
            row_idx ++;
            nn_idx  ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   = - 1.0 * traj_order/time_intervals[0];
            dA[nn_idx+1] =   1.0 * traj_order/time_intervals[0];
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;

            jcolA[nn_idx]   = i * vars_number;
            jcolA[nn_idx+1] = i * vars_number + 1;

            row_idx ++;
            nn_idx += 2;
        }
        // acceleration : 
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   =   1.0 * traj_order * (traj_order - 1)/pow(time_intervals[0],2);
            dA[nn_idx+1] = - 2.0 * traj_order * (traj_order - 1)/pow(time_intervals[0],2);
            dA[nn_idx+2] =   1.0 * traj_order * (traj_order - 1)/pow(time_intervals[0],2);
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;
            irowA[nn_idx+2] = row_idx;

            jcolA[nn_idx]   = i * vars_number;
            jcolA[nn_idx+1] = i * vars_number + 1;
            jcolA[nn_idx+2] = i * vars_number + 2;

            row_idx ++;
            nn_idx += 3;
        }
        //jerk
        

     

    //   End position 
     
        // position :
        for(int i = 0; i < 3; i++)
        {   
            dA[nn_idx]  = 1.0;
            irowA[nn_idx] = row_idx;
            jcolA[nn_idx] = nx - 1 - (2 - i) * vars_number;

            row_idx ++;
            nn_idx  ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   = - 1.0 * traj_order/pow(time_intervals[segs-1],1);
            dA[nn_idx+1] =   1.0 * traj_order/pow(time_intervals[segs-1],1);
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;

            jcolA[nn_idx]   = nx - 1 - (2 - i) * vars_number - 1;
            jcolA[nn_idx+1] = nx - 1 - (2 - i) * vars_number;

            row_idx ++;
            nn_idx += 2;
        }
        // acceleration : 
        for(int i = 0; i < 3; i++)
        { 
            dA[nn_idx]   =   1.0 * traj_order * (traj_order - 1) / pow(time_intervals[segs-1],2);
            dA[nn_idx+1] = - 2.0 * traj_order * (traj_order - 1) / pow(time_intervals[segs-1],2);
            dA[nn_idx+2] =   1.0 * traj_order * (traj_order - 1) / pow(time_intervals[segs-1],2);
            
            irowA[nn_idx]   = row_idx;
            irowA[nn_idx+1] = row_idx;
            irowA[nn_idx+2] = row_idx;

            jcolA[nn_idx]   = nx - 1 - (2 - i) * vars_number - 2;
            jcolA[nn_idx+1] = nx - 1 - (2 - i) * vars_number - 1;
            jcolA[nn_idx+2] = nx - 1 - (2 - i) * vars_number ;

            row_idx ++;
            nn_idx += 3;
        }
        //jerk
        
    

    // 关节点处的 P V A J
    
        int sub_shift = 0;
        double val0, val1;
        for(int k = 0; k < (segs - 1); k ++ )
        {   
            double scale_k = time_intervals[k];
            double scale_n = time_intervals[k+1];
            // position :
            val0 = scale_k;
            val1 = scale_n;
            for(int i = 0; i < 3; i++)
            {
                dA[nn_idx]   =  1.0;
                dA[nn_idx+1] = -1.0;
                
                irowA[nn_idx]   = row_idx;
                irowA[nn_idx+1] = row_idx;
                jcolA[nn_idx]   = sub_shift + (i+1) * vars_number - 1;//前一段的轨迹的末尾
                jcolA[nn_idx+1] = sub_shift + all_vars_number + i * vars_number;//后一段的轨迹的起点
                row_idx ++;
                nn_idx += 2;
            }
            
            //velocity

            for(int i = 0; i < 3; i++)
            {  
                dA[nn_idx]   = -1.0/val0;
                dA[nn_idx+1] =  1.0/val0;
                dA[nn_idx+2] =  1.0/val1;
                dA[nn_idx+3] = -1.0/val1;
                
                irowA[nn_idx]   = row_idx;
                irowA[nn_idx+1] = row_idx;
                irowA[nn_idx+2] = row_idx;
                irowA[nn_idx+3] = row_idx;

                jcolA[nn_idx]   = sub_shift + (i+1) * vars_number - 2;    
                jcolA[nn_idx+1] = sub_shift + (i+1) * vars_number - 1;    
                jcolA[nn_idx+2] = sub_shift + all_vars_number + i * vars_number;
                jcolA[nn_idx+3] = sub_shift + all_vars_number + i * vars_number + 1;
                
                row_idx ++;
                nn_idx += 4;
            }
            // acceleration :
            
            for(int i = 0; i < 3; i++)
            {  
                dA[nn_idx]   =  1.0  /pow(val0,2);
                dA[nn_idx+1] = -2.0  /pow(val0,2);
                dA[nn_idx+2] =  1.0  /pow(val0,2);
                dA[nn_idx+3] = -1.0  /pow(val1,2);
                dA[nn_idx+4] =  2.0  /pow(val1,2);
                dA[nn_idx+5] = -1.0  /pow(val1,2);
                
                irowA[nn_idx]   = row_idx;
                irowA[nn_idx+1] = row_idx;
                irowA[nn_idx+2] = row_idx;
                irowA[nn_idx+3] = row_idx;
                irowA[nn_idx+4] = row_idx;
                irowA[nn_idx+5] = row_idx;

                jcolA[nn_idx]   = sub_shift + (i+1) * vars_number - 3;    
                jcolA[nn_idx+1] = sub_shift + (i+1) * vars_number - 2;    
                jcolA[nn_idx+2] = sub_shift + (i+1) * vars_number - 1;    
                jcolA[nn_idx+3] = sub_shift + all_vars_number + i * vars_number;    
                jcolA[nn_idx+4] = sub_shift + all_vars_number + i * vars_number + 1;
                jcolA[nn_idx+5] = sub_shift + all_vars_number + i * vars_number + 2;
                row_idx ++;
                nn_idx += 6;
            }
            //jerk:

            for(int i = 0; i < 3; i++)
            {  
                dA[nn_idx]   =  1.0  /pow(val0,3);
                dA[nn_idx+1] = -1.0  /pow(val0,3);
                dA[nn_idx+2] = -1.0  /pow(val0,3);
                dA[nn_idx+3] =  1.0  /pow(val0,3);
                
                dA[nn_idx+4] = -1.0  /pow(val1,3);
                dA[nn_idx+5] =  1.0  /pow(val1,3);
                dA[nn_idx+6] =  1.0  /pow(val1,3);
                dA[nn_idx+7] = -1.0  /pow(val1,3);
                
                
                irowA[nn_idx]   = row_idx;
                irowA[nn_idx+1] = row_idx;
                irowA[nn_idx+2] = row_idx;
                irowA[nn_idx+3] = row_idx;
                irowA[nn_idx+4] = row_idx;
                irowA[nn_idx+5] = row_idx;
                irowA[nn_idx+6] = row_idx;
                irowA[nn_idx+7] = row_idx;
                

                jcolA[nn_idx]   = sub_shift + (i+1) * vars_number - 4;    
                jcolA[nn_idx+1] = sub_shift + (i+1) * vars_number - 3;    
                jcolA[nn_idx+2] = sub_shift + (i+1) * vars_number - 2;
                jcolA[nn_idx+3] = sub_shift + (i+1) * vars_number - 1;
                jcolA[nn_idx+4] = sub_shift + all_vars_number + i * vars_number;    
                jcolA[nn_idx+5] = sub_shift + all_vars_number + i * vars_number + 1;
                jcolA[nn_idx+6] = sub_shift + all_vars_number + i * vars_number + 2;
                jcolA[nn_idx+7] = sub_shift + all_vars_number + i * vars_number + 3;
                
                row_idx ++;
                nn_idx += 8;
            }


            sub_shift += all_vars_number;
        }
    
    //下面开启的是不等式约束
    int ieq_con_pos_num = 3*vars_number*segs; //飞行走廊的位置约束
    int high_order_con_num = 3*(vars_number-1)*segs+3*(vars_number-2)*segs;//速度 加速度约束 （不能太快）
    const int mz  = ieq_con_pos_num + high_order_con_num;//不等式约束总个数
    char iclow[mz];
    char icupp[mz];
    double clow[mz];
    double cupp[mz];

    int m_idx = 0;
    // stacking all bnounds
    for(int k = 0; k < segs; k++) 
    {   //0 for xl,1 for xu,2 for yl,3 for yu,4 for zl,5 for z
        double xl = corridor.cubes[k].borders[0];
        double xu = corridor.cubes[k].borders[1];
        double yl = corridor.cubes[k].borders[2];
        double yu = corridor.cubes[k].borders[3];
        double zl = corridor.cubes[k].borders[4];
        double zu = corridor.cubes[k].borders[5];
        for(int i=0;i<vars_number;i++){
            iclow[m_idx] = 1;
            clow[m_idx] = xl;
            icupp[m_idx] = 1;
            cupp[m_idx] = xu;
            m_idx++;//x轴
        }
        for(int i=0;i<vars_number;i++){
            iclow[m_idx] = 1;
            clow[m_idx] = yl;
            icupp[m_idx] = 1;
            cupp[m_idx] = yu;
            m_idx++;//y轴
        }
        for(int i=0;i<vars_number;i++){
            iclow[m_idx] = 1;
            clow[m_idx] = zl;
            icupp[m_idx] = 1;
            cupp[m_idx] = zu;
            m_idx++;//z轴
        }        
       
    }

    // 速度约束
    for(int i = 0; i < 3*(vars_number-1)*segs; i++)
    {
        iclow[m_idx] = 1;
        icupp[m_idx] = 1;
        clow[m_idx]  = -max_vel;
        cupp[m_idx]  = max_vel;
        m_idx++;
    }
     //加速度约束
    for(int i=0;i<3*(vars_number-2)*segs;i++){
        iclow[m_idx]=1;
        icupp[m_idx]=1;
        clow[m_idx]=-max_acc;
        cupp[m_idx]= max_acc;
        m_idx++;
    }
   
    //int nnzC = 3 * ieq_con_pos_num; //非0元素个数
    int nnzC =(vars_number*3)* segs + segs * (vars_number-1)*2 *3 + segs * (vars_number-2)*3 *3;//P+V+A
    
    int irowC[nnzC];
    int jcolC[nnzC];
    double dC[nnzC];
    nn_idx  = 0;
    row_idx = 0;
    //position
    for(int k = 0; k < segs*all_vars_number; k++)
    {
        dC[nn_idx] = 1;
        irowC[nn_idx]=row_idx;
        jcolC[nn_idx]= k;
        row_idx ++;
        nn_idx ++;
    }
    //velocity
    for(int k = 0; k < segs ; k ++ )
    {   
            for(int i = 0; i < 3; i++)
            {  // for x, y, z loop
                for(int j = 0;j < traj_order;j++){
                    dC[nn_idx]     = -1.0 * traj_order/time_intervals[k];
                    dC[nn_idx + 1] =  1.0 * traj_order/time_intervals[k];
                    irowC[nn_idx]     = row_idx;
                    irowC[nn_idx + 1] = row_idx;
                    jcolC[nn_idx]     = k * all_vars_number + i * vars_number+j;    
                    jcolC[nn_idx + 1] = k * all_vars_number + i * vars_number + j + 1;    
                    row_idx ++;
                    nn_idx += 2;
                }
            }
    }
    for(int k = 0; k < segs ; k ++ )
    {   
        double scale_k = pow(time_intervals[k],2);
        for(int i = 0; i < 3; i++)
        { 
            for(int j = 0; j < traj_order - 1; j++)
                {    
                    dC[nn_idx]     =  1.0 * traj_order * (traj_order - 1) / scale_k;
                    dC[nn_idx + 1] = -2.0 * traj_order * (traj_order - 1) / scale_k;
                    dC[nn_idx + 2] =  1.0 * traj_order * (traj_order - 1) / scale_k;

                    irowC[nn_idx]     = row_idx;
                    irowC[nn_idx + 1] = row_idx;
                    irowC[nn_idx + 2] = row_idx;

                    jcolC[nn_idx]     = k * all_vars_number + i * vars_number + j;    
                    jcolC[nn_idx + 1] = k * all_vars_number + i * vars_number + j + 1;    
                    jcolC[nn_idx + 2] = k * all_vars_number + i * vars_number + j + 2;    
                    
                    row_idx ++;
                    nn_idx += 3;
                }
            }
    }
    //在开始定义优化对象之前，首先进行准备工作定义，包括M，Q矩阵
    _Q = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3);
    _M = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3);
    for(int i=0; i<segs; i++){
        for(int j = 0; j < 3; j++){
            // calculate Matrix Q
            _Q.block(i*all_vars_number+j*vars_number, i*all_vars_number+j*vars_number, vars_number, vars_number) = getQ(vars_number, time_intervals,i);
            // calculate Matrix M   
            _M.block(i*all_vars_number+j*vars_number, i*all_vars_number+j*vars_number, vars_number, vars_number) = getM(vars_number, time_intervals, i);
        }
    }
   
    MatrixXd M_QM;
    M_QM = MatrixXd::Zero(_M.rows(),_M.cols());
    //ROS_INFO_STREAM("M:rows"<<_M.rows()<<"col"<<_M.cols()<<"Q:rows"<<_Q.rows()<<"col:"<<_Q.cols());
    M_QM = _M.transpose()*_Q*_M;//最终的二次型对应的矩阵CT*M_QM*C,_M.transpose().dot*_Q*M
    //下面开始定义优化函数
    //M_QM = MatrixXd::eye
    for(int i = 0; i < nx; i++)//一次项为0
        c[i] = 0.0;

    const int nnzQ = 3 * segs * (traj_order + 1) * (traj_order + 2) / 2; //n(n-1)/2
    int    irowQ[nnzQ]; 
    int    jcolQ[nnzQ];
    double    dQ[nnzQ];
    for(int i=0;i<nnzQ;i++){
        dQ[i]=-999;
    }
    sub_shift = 0;
    int Q_idx = 0;

    for(int k = 0; k < segs; k ++){
        double scale_k = time_intervals[k];
        for(int p = 0; p < 3; p ++ )
            for( int i = 0; i < vars_number; i ++ )
                for( int j = 0; j < vars_number; j ++ )
                    if( i >= j ){
                        irowQ[Q_idx] = sub_shift + p * vars_number + i;   
                        jcolQ[Q_idx] = sub_shift + p * vars_number + j;  
                        dQ[Q_idx] = M_QM(sub_shift + p * vars_number + i,sub_shift + p * vars_number + j);
                        //dQ[Q_idx] = 1;
                        Q_idx ++ ;
                    }
        sub_shift += all_vars_number;
    }
    // 下面开始OOQP库的求解
    //my=0;
    //nnzA=0;
    QpGenSparseMa27 * qp 
    = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
    //cout<<"irowQ: "<<irowQ[nnzQ-1]<<"jcolQ: "<<jcolQ[nnzQ-1];
    //cout<<"nx: "<<nx<<" my: "<<my<<" mz: "<<mz<<" nnzQ: "<<nnzQ<<" nnzA: "<<nnzA<<" nnzC: "<<nnzC<<" size: "<<M_QM.cols()<<" "<<M_QM.rows();
    QpGenData * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
        c,      irowQ,  nnzQ,   jcolQ,  dQ,
        xlow,   ixlow,  xupp,   ixupp,
        irowA,  nnzA,   jcolA,  dA,     b,
        irowC,  nnzC,   jcolC,  dC,
        clow,   iclow,  cupp,   icupp );

    QpGenVars      * vars  = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid = (QpGenResiduals *) qp->makeResiduals( prob );
    GondzioSolver  * s     = new GondzioSolver( qp, prob );
    
    // Turn Off/On the print of the solving process
    // s->monitorSelf();
    int ierr = s->solve(prob, vars, resid);
    ROS_INFO("ierr   %d",ierr);
    if( ierr == 0 ) 
    {
        double d_var[nx];
        vars->x->copyIntoArray(d_var);
        // cout<<"d_var="<<d_var;
        // int temp_count=0;
        // for(int kk=0;kk<nx;kk++)
        // {
        //     cout<<"d_var="<<d_var[kk];
        //     temp_count++;
        //     if(temp_count%10==0)
        //         cout<<"    count="<<temp_count<<endl;
        // }

        PolyCoeff = MatrixXd::Zero(segs, all_vars_number);
        PolyTime  = VectorXd::Zero(segs);
        obj = 0.0;
        
        int var_shift = 0;

        MatrixXd Q_o(vars_number,vars_number);
        //    int s1d1CtrlP_num = traj_order + 1;
        //    int s1CtrlP_num   = 3 * s1d1CtrlP_num;
        //int min_order_l = floor(minimize_order);
        //int min_order_u = ceil (minimize_order);

        for(int i = 0; i < segs; i++ )
        {   
            PolyTime(i) = time_intervals[i];

            for(int j = 0; j < all_vars_number; j++)
                {
                    PolyCoeff(i , j) = d_var[j + var_shift];
                    // cout<<"coeff in is  "<<PolyCoeff(i , j)<<"i="<<i<<"  j="<<j<<endl;
                }
            var_shift += all_vars_number;     
        } 

    } 
    else if( ierr == 3)
        cout << "The program is provably infeasible, check the formulation.\n";
    else if (ierr == 4)
        cout << "The program is very slow in convergence, may have numerical issue.\n";
    else
        cout << "Solver numerical error.\n";
    
    return ierr;
    //return 1;
}
