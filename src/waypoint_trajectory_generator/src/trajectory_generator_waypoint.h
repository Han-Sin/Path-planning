#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
    private:
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
        

#endif
