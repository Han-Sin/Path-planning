#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
    private:
		double _qp_cost;
		Eigen::MatrixXd _Q,_M,_Ct;
		Eigen::VectorXd _Px, _Py, _Pz;
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
};
        

#endif
