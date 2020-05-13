#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
//#include <waypoint_trajectory_generator/Trajectoy.h>
//#include <waypoint_trajectory_generator/trajpoint.h>
//#include <quadrotor_msgs/PolynomialTrajectory.h>
// Useful customized headers
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

// Param from launch file
    double _vis_traj_width;
    double _Vel, _Acc;
    int    _dev_order, _min_order;    
// Set the obstacle map
    double _resolution, _inv_resolution;
    double _x_size, _y_size, _z_size;
    Vector3d _map_lower, _map_upper;
    int _max_x_id, _max_y_id, _max_z_id;
//
    TrajectoryGeneratorWaypoint * _trajGene     = new TrajectoryGeneratorWaypoint();
// ros related
    ros::Subscriber _way_pts_sub,_way_pts_sub2;
    ros::Publisher  _wp_traj_vis_pub,_wp_traj_vis_pub2, _wp_path_vis_pub,  _vel_pub,_acc_pub,_points_pub;

// for planning
    int _poly_num1D;
    MatrixXd _polyCoeff;
    VectorXd _polyTime;
    Vector3d _startPos = Vector3d::Zero();
    Vector3d _startVel = Vector3d::Zero();

// declare
    void visWayPointTraj( MatrixXd polyCoeff, VectorXd time,int flag);
    void visWayPointPath(MatrixXd path);
    Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t );
	Vector3d getVelocity(MatrixXd polyCoeff, int k, double t);
    Vector3d getAcc(MatrixXd polyCoeff, int k, double t);
    VectorXd timeAllocation( MatrixXd Path);
    void trajGeneration(Eigen::MatrixXd path,int flag);
    void rcvWaypointsCallBack(const nav_msgs::Path & wp);
    void rcvWaypointsCallBack2(const nav_msgs::Path & wp);
    nav_msgs::Path vector3d_to_waypoints(vector<Vector3d> path);



void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    //MatrixXd waypoints(wp_list.size() + 1, 3);
    //waypoints.row(0) = _startPos;
    MatrixXd waypoints(wp_list.size(), 3);
    //for(int k = 0; k < (int)wp_list.size(); k++)
    //    waypoints.row(k+1) = wp_list[k];
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];
    
    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    trajGeneration(waypoints,0);
    //化简后的结点。
}


void rcvWaypointsCallBack2(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    //MatrixXd waypoints(wp_list.size() + 1, 3);
    //waypoints.row(0) = _startPos;
    MatrixXd waypoints(wp_list.size(), 3);
    //for(int k = 0; k < (int)wp_list.size(); k++)
    //    waypoints.row(k+1) = wp_list[k];
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];
    
    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    trajGeneration(waypoints,1);
}


void trajGeneration(Eigen::MatrixXd path,int flag=0)
{   
    
    
    MatrixXd vel = MatrixXd::Zero(2, 3); 
    MatrixXd acc = MatrixXd::Zero(2, 3);

    vel.row(0) = _startVel;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    _polyTime  = timeAllocation(path);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
    visWayPointPath(path);
    visWayPointTraj( _polyCoeff, _polyTime,flag);
    int unsafe_segment;
    if(!flag){
        //开始迭代
        MatrixXd repath = path;
        int count = 0;
        unsafe_segment = _trajGene->safeCheck(_polyCoeff, _polyTime);
        while(unsafe_segment != -1){
        //cout << "reoptimize!"<< endl;
            MatrixXd repath_a,repath_b;
            VectorXd mid(3);
            int rows = repath.rows();
            mid(0) = (repath(unsafe_segment,0) + repath(unsafe_segment+1,0))/2;
            mid(1) = (repath(unsafe_segment,1) + repath(unsafe_segment+1,1))/2;
            mid(2) = (repath(unsafe_segment,2) + repath(unsafe_segment+1,2))/2;
            repath_a = repath.block(0,0,unsafe_segment+1,3);
            repath_b = repath.block(unsafe_segment+1,0,rows-unsafe_segment-1,3);
            repath.resize(rows + 1,3);
            repath.block(0,0,unsafe_segment+1,3) = repath_a;
            repath(unsafe_segment+1,0) = mid(0);
            repath(unsafe_segment+1,1) = mid(1);
            repath(unsafe_segment+1,2) = mid(2);
            repath.block(unsafe_segment+2,0,rows-unsafe_segment-1,3) = repath_b;
            _polyTime  = timeAllocation(repath);
            _polyCoeff = _trajGene->PolyQPGeneration(_dev_order, repath, vel, acc, _polyTime);
            unsafe_segment = _trajGene->safeCheck(_polyCoeff, _polyTime);
            count++;
            if(count > 10)
                unsafe_segment = -1;
            visWayPointTraj( _polyCoeff, _polyTime,flag);
        }
    visWayPointTraj( _polyCoeff, _polyTime,flag);
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel",   _Vel,   1.0 );
    nh.param("planning/acc",   _Acc,   1.0 );//平均时速
    nh.param("planning/dev_order", _dev_order,  4 );
    nh.param("planning/min_order", _min_order,  3 );
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );

    //_poly_numID is the maximum order of polynomial
    _poly_num1D = 2 * _dev_order;

    //state of start point  
    _startVel(0)  = 0;
    _startVel(1)  = 0;
    _startVel(2)  = 0;
    
    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );
    _way_pts_sub2    = nh.subscribe( "/demo_node/simplified_waypoints2", 1, rcvWaypointsCallBack2 );

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_traj_vis_pub2 = nh.advertise<visualization_msgs::Marker>("vis_trajectory2", 1);

    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);
    _vel_pub =         nh.advertise<nav_msgs::Path>("vel",1);
    _acc_pub =         nh.advertise<nav_msgs::Path>("acc",1);
    //_points_pub = nh.advertise<waypoint_trajectory_generator::Trajectoy>("trajectory_points",1);
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;  
    _inv_resolution = 1.0 / _resolution;
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
    _trajGene-> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();  
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time,int flag)
{        
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "world";
    //waypoint_trajectory_generator::Trajectoy _traj;
    //_traj.traj_points.clear();
    _traj_vis.ns = "traj_node/trajectory_waypoints";
    // else if(flag==1)
        // _traj_vis.ns = "traj_node/trajectory_waypoints2";

    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    if(flag)
    {
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;
    }

    else if(flag==0)
    {
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 1.0;
    }

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    vector<Vector3d> vel_pub;
    vector<Vector3d> acc_pub;

    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
            // if(t==0.0||t+0.01>=time(i))
            if(true)
            {
                Vector3d vel = getVelocity(polyCoeff, i, t);
                // ROS_INFO_STREAM("time: "<<t<<"      VX= " << vel(0) << "     VY= " << vel(1) << "     VZ= " << vel(2));
                // ROS_INFO("time=%f",t);
                Vector3d acc = getAcc(polyCoeff,i,t);
                vel_pub.push_back(vel);
                acc_pub.push_back(acc);
                // if(t+0.01>=time(i))
                //     ROS_INFO(" ");
            }
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);
          waypoint_trajectory_generator::trajpoint traj_point;
          traj_point.seg = i;
          traj_point.point = pt;
          _traj.traj_points.push_back(traj_point);
          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }
    ROS_INFO_STREAM("optimizer traj sucess, the length is "<<traj_len);
    if(flag)
        _wp_traj_vis_pub.publish(_traj_vis);
    else if(flag==0)
        _wp_traj_vis_pub2.publish(_traj_vis);
    _vel_pub.publish(vector3d_to_waypoints(vel_pub));
    _acc_pub.publish(vector3d_to_waypoints(acc_pub));
    _points_pub.publish(_traj);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "world";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = _vis_traj_width;
    line_list.scale.y = _vis_traj_width;
    line_list.scale.z = _vis_traj_width;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

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
Vector3d getVelocity(MatrixXd polyCoeff, int k, double t)
{
	Vector3d vel;
	for (int dim = 0; dim < 3; dim++) {
		VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
		VectorXd time = VectorXd::Zero(_poly_num1D);
		for (int j = 0; j < _poly_num1D; j++)
			if (j == 0)
				time(j) = 0;
			else if (j == 1)
				time(j) = 1;
			else
				time(j) = pow(t, j-1)*j;

		vel(dim) = coeff.dot(time);
	}
	return vel;
}
Vector3d getAcc(MatrixXd polyCoeff, int k, double t){
    Vector3d Acc;
	for (int dim = 0; dim < 3; dim++) {
		VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
		VectorXd time = VectorXd::Zero(_poly_num1D);
		for (int j = 0; j < _poly_num1D; j++)
			if (j == 0 || j==1)
				time(j) = 0;
			else if (j == 2)
				time(j) = 2;
			else
				time(j) = pow(t, j-2)*j*(j-1);

		Acc(dim) = coeff.dot(time);
	}
	return Acc;
}

VectorXd timeAllocation( MatrixXd Path)
{ 

    VectorXd time(Path.rows() - 1);

    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        if(x2<=0){
            time(i) = 2*sqrt(distance/_Acc);
        }
        else{
            double t1 = _Vel / _Acc;
            double t2 = x2 / _Vel;
            time(i) = 2 * t1 + t2;;
        }
        //time(i) = distance/_Vel;
    }
    return time;
}


// VectorXd timeAllocation( MatrixXd Path)
// { 
//     VectorXd time(Path.rows() - 1);

//     for (int k = 0; k < (Path.rows() - 1); k++)
//     {
//         double dtxyz;

//         Vector3d p0   = Path.row(k);        
//         Vector3d p1   = Path.row(k + 1);    
//         double D    = (p1 - p0).norm();             

//         double acct = (_Vel) / _Acc;
//         double accd = (_Acc * acct * acct / 2);
//         double dcct = _Vel / _Acc;                                  
//         double dccd = _Acc * dcct * dcct / 2;                           

//         if (D < accd + dccd)
//         {   
//             double t1 = sqrt( _Acc * D ) / _Acc;
//             double t2 = (_Acc * t1) / _Acc;
//             dtxyz     = t1 + t2;    
//         }
//         else
//         {                                        
//             double t1 = acct;                              
//             double t2 = (D - accd - dccd) / _Vel;
//             double t3 = dcct;
//             dtxyz     = t1 + t2 + t3;                                                                  
//         }

//         time(k) = dtxyz;
//     }

//     return time;
// }

nav_msgs::Path vector3d_to_waypoints(vector<Vector3d> path)
{
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    for (auto ptr: path)
    {
        pt.pose.position.y =  ptr(1);
        pt.pose.position.x =  ptr(0);
        pt.pose.position.z =  ptr(2);
        waypoints.poses.push_back(pt);//维护waypoints
    }
    return waypoints;
}