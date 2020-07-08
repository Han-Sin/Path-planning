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
#include "RRTPathSearch.h"
#include "hybridAstar_searcher.h"
double _vis_traj_width;
using namespace std;
using namespace Eigen;
//Vector3d _back_drone_pos(0,0,2);
namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    
int is_dynamic,is_track;
// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;
// ros related
ros::Subscriber _map_sub, _pts_sub,_traj_sub;

ros::Publisher   _grid_map_vis_pub,hybridAstar_traj_vis_pub,hybridAstar_traj_path_pub,_vel_pub;



//ros::Publisher _grid_path_vis_pub,_visited_nodes_vis_pub,_grid_path_pub,_grid_path_pub2,_simplified_waypoints_pub
hybridAstar_searcher * hybridAstar_path_finder     = new hybridAstar_searcher();
void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void visGridPath( vector<Vector3d> nodes, int alogrithm_choice );
void visVisitedNode( vector<Vector3d> nodes,int alogrithm_choice);
void hybridAstar_traj_generator(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
           Eigen::Vector3d end_pt, Eigen::Vector3d end_v);
nav_msgs::Path vector3d_to_waypoints(vector<Vector3d> path);


void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;
    ROS_INFO("[node] receive the planning target");
    Vector3d start_v(0,0,0);
    Vector3d start_a(1,1,1);
    Vector3d end_v(0,0,0);
    hybridAstar_traj_generator(_start_pt, start_v, start_a,
            target_pt, end_v);
    _start_pt = target_pt;
    ROS_WARN("1111111111");
    ROS_INFO_STREAM("TARGET!!: "<<target_pt);
    
 
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{
    if(_has_map&&!is_dynamic ) 
    {
        ROS_WARN("map already had");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    ROS_INFO("cloud points size=%d\n",(int)cloud.points.size());
    hybridAstar_path_finder->resetObs();
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];
        // ROS_INFO("cloud points x=%f  y=%f   z=%f   \n",pt.x, pt.y, pt.z);
        // set obstalces into grid map for path planning
        hybridAstar_path_finder->setObs(pt.x, pt.y, pt.z);
        // for visualize only
        Vector3d cor_round = hybridAstar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
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

void visHybridAstar(vector<Vector3d> state_list){
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "world";
    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    //_vis_traj_width = 0.05;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;//红色线
    //double traj_len = 0.0;
    //int count = 0;
    //Vector3d cur, pre;
    //cur.setZero();
    //pre.setZero();
    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;
    for(int i=0;i<state_list.size();i++){
        pos  = state_list[i];
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _traj_vis.points.push_back(pt);
    }
    hybridAstar_traj_vis_pub.publish(_traj_vis);
}

void hybridAstar_traj_generator(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
           Eigen::Vector3d end_pt, Eigen::Vector3d end_v)
{
    //Call A* to search for a path

    ros::Time time_1 = ros::Time::now();
    hybridAstar_path_finder->search(start_pt, start_v,  start_a,
           end_pt, end_v,hybridAstar_traj_path_pub);
    vector<Vector3d> state_list = hybridAstar_path_finder->getKinoTraj(0.01);
    visHybridAstar(state_list);
    vector<Vector3d> vel_xyz;
    vel_xyz = hybridAstar_path_finder->getKinoVel(0.001);
    //for(int i=0;i<vel_xyz.size();i++){
    //    cout<<"vel x: "<<vel_xyz[i](0)<<"vel y: "<<vel_xyz[i](1)<<"vel z: "<<vel_xyz[i](2)<<endl;
    //}
    _vel_pub.publish(vector3d_to_waypoints(vel_xyz));
    hybridAstar_path_finder->resetUsedGrids();

    /*if(flag==1)
    {
        auto visited_nodes = _astar_path_finder->getVisitedNodes();
        auto simplified_points_path = _astar_path_finder->getSimplifiedPoints(1000);//化简后的关键点（100指不中间采样）
        nav_msgs::Path simplified_waypoints=_astar_path_finder->vector3d_to_waypoints(simplified_points_path);
        _simplified_waypoints_pub.publish(simplified_waypoints);//发布关键点
        _grid_path_pub.publish(_astar_path_finder->vector3d_to_waypoints(grid_path));//发布未化简的A×路径
        ros::Time time_2 = ros::Time::now();
        visGridPath (grid_path, 0);//可视化搜寻的栅格地图
        visVisitedNode(visited_nodes,0);
    }
    else if(flag==2)
    {
        visGridPath(grid_path,1);
        _grid_path_pub2.publish(_astar_path_finder->vector3d_to_waypoints(grid_path));//发布未化简的A×路径
    }
        _astar_path_finder->resetUsedGrids();
    */


}






int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    hybridAstar_traj_vis_pub      = nh.advertise<visualization_msgs::Marker>("vis_hybridAstar_traj", 1);
    hybridAstar_traj_path_pub = nh.advertise<visualization_msgs::Marker>("vis_path_traj", 1);
     _vel_pub =         nh.advertise<nav_msgs::Path>("vel",1);//速度发送
    /*_grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);//可视化栅格路径
     _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
     _grid_path_pub                = nh.advertise<nav_msgs::Path>("grid_path",50);//发布优化后的轨迹
     _grid_path_pub2               = nh.advertise<nav_msgs::Path>("grid_path2",50);//发布优化后的轨迹
    */
    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    nh.param("is_dynamic",is_dynamic,0);
    nh.param("is_track",is_track,0);
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    hybridAstar_path_finder->setParam(nh);
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;//-25,-25,0
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;//25,25,0
    _inv_resolution = 1.0 / _resolution;//5
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
    hybridAstar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    

    ros::Rate rate(100);
    bool status = ros::ok();

    
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
        
    }

    delete hybridAstar_path_finder;
    return 0;
}
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



/*
void visGridPath( vector<Vector3d> nodes, int alogrithm_choice )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    switch(alogrithm_choice)
    {
        case 0://for A*
        {
            node_vis.color.a = 1.0;
            node_vis.color.r = 0.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 0.0;
            break;
        }
        case 1:
        {
            node_vis.color.a = 1.0;
            node_vis.color.r = 1.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 0.0;
            break;
        }
        case 2:
        {
            node_vis.color.a = 1.0;
            node_vis.color.r = 0.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 1.0;
        }

    }
    
    // if(is_use_jps)
    //     node_vis.ns = "demo_node/jps_path";
    // else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    



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


    switch(alogrithm_choice)
    {
        case 0://for A*
        {
            _grid_path_vis_pub.publish(node_vis);
            break;
        }

    }
}

void visVisitedNode(vector<Vector3d> nodes,int alogrithm_choice)
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

    switch(alogrithm_choice)
    {
        case 0://for A*
        {
            node_vis.color.a = 0.15;
            node_vis.color.r = 0.0;
            node_vis.color.g = 1.0;
            node_vis.color.b = 0.0;
            break;
        }

        case 1://for JPS
        {
            node_vis.color.a = 0.4;
            node_vis.color.r = 0.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 1.0;
            break;
        }
        
        case 2://for RRT
        {
            node_vis.color.a = 0.8;
            node_vis.color.r = 0.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 1.0;
            break;
        }
    }
    



    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    switch(alogrithm_choice)
    {
        case 0://for A*
        {
            _visited_nodes_vis_pub.publish(node_vis);
            break;
        }
    }
}

*/



