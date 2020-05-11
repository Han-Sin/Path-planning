#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Astar_searcher.h"
#include "JPS_searcher.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub,_traj_sub;
ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub,_simplified_waypoints_pub;

AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
JPSPathFinder   * _jps_path_finder       = new JPSPathFinder();


visualization_msgs::Marker traj_global;

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );
void visVisitedNode( vector<Vector3d> nodes );
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt); 
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    ROS_INFO("cloud points size=%d\n",(int)cloud.points.size());
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];
        // ROS_INFO("cloud points x=%f  y=%f   z=%f   \n",pt.x, pt.y, pt.z);
        // set obstalces into grid map for path planning
        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        _jps_path_finder->setObs(pt.x, pt.y, pt.z);

        // for visualize only
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    //Call A* to search for a path
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    ros::Time time_1 = ros::Time::now();

    //Retrieve the path
    auto grid_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    // auto turning_points_path = _astar_path_finder->getTurningPoints();

    // auto simplified_points_path          = _astar_path_finder->pathSimplify(grid_path, 0.05);//RDP算法化简
    auto simplified_points_path = _astar_path_finder->getSimplifiedPoints();//化简后的关键点
    // auto simplified_points_path = _astar_path_finder->getSimplifiedPoints_by_lines();//化简后的关键点,直线查找

    nav_msgs::Path simplified_waypoints=_astar_path_finder->vector3d_to_waypoints(simplified_points_path);
    // nav_msgs::Path simplified_waypoints=_astar_path_finder->vector3d_to_waypoints(grid_path);

    //迭代加关键点
    auto temp_path=simplified_points_path;
    int collision_flag=1;
    while(collision_flag)
    {
        _simplified_waypoints_pub.publish(_astar_path_finder->vector3d_to_waypoints(temp_path));
        ros::Rate rate(50);//等待traj_generator_node发送回来traj
        rate.sleep();
        temp_path=_astar_path_finder->recursive_get_simplified_points(temp_path,traj_global,collision_flag);
        visVisitedNode(temp_path);
    }
    _simplified_waypoints_pub.publish(_astar_path_finder->vector3d_to_waypoints(temp_path));


    // _simplified_waypoints_pub.publish(simplified_waypoints);
    // _simplified_waypoints_pub.publish(_astar_path_finder->vector3d_to_waypoints(simplified_path_RDP));

    ros::Time time_2 = ros::Time::now();
    ROS_WARN("Total time cost is %f ms", (time_2 - time_1).toSec() * 1000.0);

    //Visualize the result
    visGridPath (grid_path, false);
    // visVisitedNode(visited_nodes);
    // visVisitedNode(simplified_points_path);
    // visVisitedNode(simplified_path_RDP);

    //Reset map for next call
    _astar_path_finder->resetUsedGrids();

    //_use_jps = 0 -> Do not use JPS
    //_use_jps = 1 -> Use JPS
    //you just need to change the #define value of _use_jps
#define _use_jps 0
#if _use_jps
    {
        //Call JPS to search for a path
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, _use_jps);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
    }
#endif
}


void TrajectoryCallBack(const visualization_msgs::Marker  & trajectory)
{
    traj_global=trajectory;
    ROS_WARN("traj_update!");
    // auto points=trajectory.points;
    // ROS_WARN("trajectory length=   ",points.size());
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _traj_sub = nh.subscribe( "/trajectory_generator_node/vis_trajectory", 1, TrajectoryCallBack );

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);

    //waypoints发布者，没写完
    _simplified_waypoints_pub     = nh.advertise<nav_msgs::Path>("simplified_waypoints",50);

    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;//-25,-25,0
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;//25,25,0
    
    _inv_resolution = 1.0 / _resolution;//5
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _astar_path_finder  = new AstarPathFinder();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    _jps_path_finder    = new JPSPathFinder();
    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    





    ros::Rate rate(100);
    bool status = ros::ok();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    
    while(status) 
    {
        // ros::spinOnce();      
        // status = ros::ok();
        // rate.sleep();
        spinner.start();
        ros::waitForShutdown();
    }

    delete _astar_path_finder;
    delete _jps_path_finder;
    return 0;
}

void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    

    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 1.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;
    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.8;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}