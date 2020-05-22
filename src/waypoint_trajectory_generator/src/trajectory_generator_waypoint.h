#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>
#include "GridNode.h"
// #include <grid_path_searcher/node.h>

class FlightCube;
class FlightCorridor;
using namespace std;

class TrajectoryGeneratorWaypoint {
    public:
		double _qp_cost;
		Eigen::MatrixXd _Q,_M,_Ct;
		Eigen::VectorXd _Px, _Py, _Pz;
    uint8_t * data;
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;
		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;
    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
    public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();
        
        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);
        Eigen::MatrixXd getQ(
          const int p_num1d, 
          const int d_order, 
          const Eigen::VectorXd &Time, 
          const int seg_index);
        Eigen::MatrixXd getM(
        const int p_num1d,
        const int d_order, 
        const Eigen::VectorXd &Time, 
        const int seg_index);
        int Factorial(int x);
        Eigen::MatrixXd getCt(
        const int seg_num, 
        const int d_order);
        Eigen::VectorXd QP_optimization(const Eigen::MatrixXd &Q,
                                        const Eigen::MatrixXd &M,
                                        const Eigen::MatrixXd &Ct,
                                        const Eigen::VectorXd &waypoint,
                                        const Eigen::VectorXd &beginstate,
                                                                  const Eigen::VectorXd &endstate,
                                                                  const int seg_num,
                                                                  const int d_order);
        void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		    void setObs(const double coord_x, const double coord_y, const double coord_z);
        Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
		    int safeCheck( Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);
        Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
};
        



class FlightCube
{
public:
  GridNodePtr start_node;
  GridNodePtr end_node;
  //           ->x_pos
  //           y_pos
  double x_pos;
  double x_neg;
  double y_pos;
  double y_neg;
  double z_pos;
  double z_neg;
  int x_pos_int;
  int x_neg_int;
  int y_pos_int;
  int y_neg_int;
  int z_pos_int;
  int z_neg_int;
  int borders_int[6];//0 for xl,1 for xu,2 for yl,3 for yu,4 for zl,5 for z
  double borders[6];



  FlightCube(GridNodePtr s_n,GridNodePtr e_n)
  {
    start_node=s_n;
    end_node=e_n;
    if(end_node->index[0]>start_node->index[0])//init x
    {
      x_pos_int=end_node->index[0]-start_node->index[0];
      x_neg_int=0;
    }
    else
    {
      x_neg_int=start_node->index[0]-end_node->index[0];
      x_pos_int=0;
    }

    if(end_node->index[1]>start_node->index[1])//init y
    {
      y_pos_int=end_node->index[1]-start_node->index[1];
      y_neg_int=0;
    }
    else
    {
      y_neg_int=start_node->index[1]-end_node->index[1];
      y_pos_int=0;
    }

    if(end_node->index[2]>start_node->index[2])//init z
    {
      z_pos_int=end_node->index[2]-start_node->index[2];
      z_neg_int=0;
    }
    else
    {
      z_neg_int=start_node->index[2]-end_node->index[2];
      z_pos_int=0;
    }
  }

  void Display()
  {
    ROS_INFO("start_node_x_int=%d   y_int=%d   z_int=%d     x_pos_int=%d  y_pos_int=%d  z_pos_int=%d  x_neg_int=%d  y_neg_int=%d  z_neg_int=%d",
    start_node->index[0],start_node->index[1],start_node->index[2],x_pos_int,y_pos_int,z_pos_int,x_neg_int,y_neg_int,z_neg_int);

    ROS_INFO("start_node_x=%f   y=%f  z=%f     x_pos=%f  y_pos=%f  z_pos=%f  x_neg=%f  y_neg=%f  z_neg=%f",
    start_node->coord[0],start_node->coord[1],start_node->coord[2],x_pos,y_pos,z_pos,x_neg,y_neg,z_neg);


    for (int i=0;i<6;i++)
      ROS_INFO("border=%f",borders[i]);
  }

};




class FlightCorridor:public TrajectoryGeneratorWaypoint
{
public:
  vector<FlightCube> cubes;
  // int max_expand_size;

  bool check_cube_safe(FlightCube cube);

  FlightCube expand_cube(FlightCube &cube);

  void update_attributes(FlightCube &cube);

};


#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>


class spatialTrajOptimizer 
{
    private:
        double obj;
        Eigen::MatrixXd PolyCoeff;
        Eigen::VectorXd PolyTime;
        MatrixXd M;
        int traj_order;

    public:
        spatialTrajOptimizer(int order){
          if(order ==6){
            M << 1,   0,   0,   0,   0,  0,  0,
					      -6,   6,   0,   0,   0,  0,  0,
					      15, -30,  15,   0,   0,  0,  0,
				        -20,  60, -60,  20,   0,  0,  0,
				        15, -60,  90, -60,  15,  0,  0,
				        -6,  30, -60,  60, -30,  6,  0,
				        1,  -6,  15, -20,  15, -6,  1;}
          else if(order==7){
            M << 1,    0,    0,    0,    0,   0,   0,   0,
				        -7,    7,    0,    0,    0,   0,   0,   0,
				        21,  -42,   21,    0,    0,   0,   0,   0,
				        -35,  105, -105,   35,    0,   0,   0,   0, 
				        35, -140,  210, -140,   35,   0,   0,   0,
				        -21,  105, -210,  210, -105,  21,   0,   0,
				        7,  -42,  105, -140,  105, -42,   7,   0,
				        -1,    7,  -21,   35,  -35,  21,  -7,   1;
          }
        }
        ~spatialTrajOptimizer(){}

        int bezierCurveGeneration( 
        const FlightCorridor &corridor,
        const int traj_order,
        const double max_vel, 
        const double max_acc);

        Eigen::MatrixXd getPolyCoeff()
        {
            return PolyCoeff;
        };

        Eigen::VectorXd getPolyTime()
        {
            return PolyTime;
        };

        double getObjective()
        {
            return obj;
        };
};







#endif
