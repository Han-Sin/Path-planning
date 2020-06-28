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
//#include <waypoint_trajectory_generator/trajpoint.h>
//#include <waypoint_trajectory_generator/Trajectoy.h>
// #include <waypoint_trajectory_generator/trajpoint.h>

using namespace std;
using namespace Eigen;
Vector3d _back_drone_pos(0,0,2);
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
Vector3d target_pt_front;
int _max_x_id, _max_y_id, _max_z_id;
bool update_map=0;


// ros related
ros::Subscriber _map_sub, _pts_sub,_traj_sub,drone_pos_sub,_back_drone_pos_sub;

ros::Publisher  _grid_path_vis_pub,_grid_path_vis_pub_jps,_grid_path_vis_pub_rrt,
 _visited_nodes_vis_pub,_visited_nodes_jps_vis_pub,_visited_nodes_rrt_vis_pub,
  _grid_map_vis_pub,_simplified_waypoints_pub,_grid_path_pub,_grid_path_pub2,front_target_pub;

AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
JPSPathFinder   * _jps_path_finder       = new JPSPathFinder();
RRTPathSearch   *  _rrt_path_finder  = new RRTPathSearch();
void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);

void vis_node_line_Path(vector<Vector3d> nodes);
void visGridPath( vector<Vector3d> nodes, int alogrithm_choice );
void visVisitedNode( vector<Vector3d> nodes,int alogrithm_choice);
void pathFinding(const Vector3d start_pt, const Vector3d target_pt,int flag=1);

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;
    // target_pt<<-4.886, -4.468, 2.500;
    target_pt_front = target_pt;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt); 
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.position.y =  target_pt(1);
    pt.pose.position.x =  target_pt(0);
    pt.pose.position.z =  target_pt(2);
    waypoints.poses.push_back(pt);//维护waypoints
    front_target_pub.publish(waypoints);
    
        
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
    _astar_path_finder->resetObs();
    _jps_path_finder->resetObs();
    _rrt_path_finder->resetObs();
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];
        // ROS_INFO("cloud points x=%f  y=%f   z=%f   \n",pt.x, pt.y, pt.z);
        // set obstalces into grid map for path planning
        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        _jps_path_finder->setObs(pt.x, pt.y, pt.z);
        _rrt_path_finder->setObs(pt.x,pt.y,pt.z);
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

    if(_astar_path_finder->coord2gridIndex(_start_pt)!=_astar_path_finder->coord2gridIndex(target_pt_front)){
       pathFinding(_start_pt,target_pt_front);
    }
    
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt,int flag)
{
    //Call A* to search for a path

    ros::Time time_1 = ros::Time::now();

    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
    //Retrieve the path
    auto grid_path     = _astar_path_finder->getPath();
    
    if(flag==1)
    {
        auto visited_nodes = _astar_path_finder->getVisitedNodes();
        auto simplified_points_path = _astar_path_finder->getSimplifiedPoints(1000);//化简后的关键点（100指不中间采样）
        nav_msgs::Path simplified_waypoints=_astar_path_finder->vector3d_to_waypoints(simplified_points_path);
        _simplified_waypoints_pub.publish(simplified_waypoints);//发布关键点
        _grid_path_pub.publish(_astar_path_finder->vector3d_to_waypoints(grid_path));//发布未化简的A×路径
        
        // _simplified_waypoints_pub3.publish(_astar_path_finder->vector3d_to_waypoints(temp_path));
        // visVisitedNode(simplified_points_path);//可视化关键点
        // _simplified_waypoints_pub.publish(simplified_waypoints);
        // _simplified_waypoints_pub.publish(_astar_path_finder->vector3d_to_waypoints(simplified_path_RDP));
        ros::Time time_2 = ros::Time::now();
        // ROS_WARN("Total time cost is %f ms", (time_2 - time_1).toSec() * 1000.0);
        //Visualize the result
        visGridPath (grid_path, 0);//可视化搜寻的栅格地图
        visVisitedNode(visited_nodes,0);
        // visVisitedNode(simplified_points_path);
        // visVisitedNode(simplified_path_RDP);
    }
    else if(flag==2)
    {
        visGridPath(grid_path,1);
        _grid_path_pub2.publish(_astar_path_finder->vector3d_to_waypoints(grid_path));//发布未化简的A×路径
    }
        
    
    //Reset map for next call
    _astar_path_finder->resetUsedGrids();
    //_use_jps = 0 -> Do not use JPS
    //_use_jps = 1 -> Use JPS
    //you just need to change the #define value of _use_jps

#define _use_jps 0
#if _use_jps
    {
        // ROS_INFO("Using JPS!");
        //Call JPS to search for a path
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, 1);
        visVisitedNode(visited_nodes,1);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
        
    }
#endif

#define _use_rrt 0
#if _use_rrt
    {
        _rrt_path_finder->RRTSearch(start_pt,target_pt);
        auto grid_path     = _rrt_path_finder->getPath();
        auto visited_nodes = _rrt_path_finder->getVisitedNodes();

        //Visualize the result
        // visGridPath   (grid_path, 2);//2 for RRT
        visVisitedNode(visited_nodes,2);
        vis_node_line_Path(grid_path);

        //Reset map for next call
        _rrt_path_finder->resetUsedGrids();
    }
#endif


}

void rcvDronePosCallBack(const visualization_msgs::Marker &pos_msg)
{
    // ROS_INFO("rcv pos!");
    // ROS_INFO("%f",pos_msg.points[0].x);

    Vector3d current_pt(pos_msg.points[0].x,pos_msg.points[0].y,pos_msg.points[0].z);
    _start_pt=current_pt;

    Vector3d offset(1,1,2);
        
    // ROS_INFO("FRONT DRONE POS = %f  %f  %f  A*",_start_pt(0),_start_pt(1),_start_pt(2));
    // ROS_INFO("TARGET DRONE POS = %f  %f  %f",target(0),target(1),target(2));
    offset=1*(_back_drone_pos-_start_pt)/(_start_pt-_back_drone_pos).norm();
    Vector3d target=_astar_path_finder->target_point_generator(_start_pt,offset);
    vector<Vector3d> temp_v;
    temp_v.push_back(target);
    visVisitedNode(temp_v,2);


    static int count=0;
    count++;
    static ros::Time time_1 = ros::Time::now();
    if(count%25==0)
        {
            ros::Time time_2 = ros::Time::now();
            //ROS_WARN("time passed %f ms", (time_2 - time_1).toSec() * 1000.0);
            time_1=time_2;
            if(is_track)
            {
                //ROS_INFO("BACK_DRONE_POS=%f %f %f ",_back_drone_pos[0],_back_drone_pos[1],_back_drone_pos[2]);
                pathFinding(_back_drone_pos,target,2);  
            }
        
        }
        
}



void rcvBackDronePosCallBack(const visualization_msgs::Marker & p)
{
    auto temp_p=p.points[0];
    _back_drone_pos<<temp_p.x,temp_p.y,temp_p.z;
    //ROS_INFO("BACK DRONE POS = %f  %f  %f",_back_drone_pos(0),_back_drone_pos(1),_back_drone_pos(2));
}

int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    drone_pos_sub = nh.subscribe("/drone_node/drone_pos",50,rcvDronePosCallBack);
    _back_drone_pos_sub = nh.subscribe( "/drone_node/drone2_pos", 1, rcvBackDronePosCallBack );
    ROS_INFO("BACK_DRONE_POS=%f %f %f ",_back_drone_pos[0],_back_drone_pos[1],_back_drone_pos[2]);
    _back_drone_pos<<0,0,2;
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _grid_path_vis_pub_jps        = nh.advertise<visualization_msgs::Marker>("grid_path_vis_jps", 1);
    _grid_path_vis_pub_rrt        = nh.advertise<visualization_msgs::Marker>("grid_path_vis_rrt", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
    _visited_nodes_jps_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_jps_vis", 1);
    _visited_nodes_rrt_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_rrt_vis", 1);
    _simplified_waypoints_pub     = nh.advertise<nav_msgs::Path>("simplified_waypoints",50);//发布优化后的轨迹
    _grid_path_pub                = nh.advertise<nav_msgs::Path>("grid_path",50);//发布优化后的轨迹
    _grid_path_pub2                = nh.advertise<nav_msgs::Path>("grid_path2",50);//发布优化后的轨迹
    front_target_pub = nh.advertise<nav_msgs::Path>("front_target",50);
    
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
    target_pt_front = _start_pt;
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
    
    _rrt_path_finder  = new RRTPathSearch();
    
    _rrt_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    

    ros::Rate rate(100);
    bool status = ros::ok();

    // ros::AsyncSpinner spinner(4); // Use 4 threads
    
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
        // spinner.start();
        // status = ros::ok();
        // ros::waitForShutdown();
        nav_msgs::Path waypoints;
        geometry_msgs::PoseStamped pt;
        pt.pose.position.y =  target_pt_front(1);
        pt.pose.position.x =  target_pt_front(0);
        pt.pose.position.z =  target_pt_front(2);
        waypoints.poses.push_back(pt);//维护waypoints
        front_target_pub.publish(waypoints);
        
    }
    // spinner.stop();

    delete _astar_path_finder;
    delete _jps_path_finder;
    delete _rrt_path_finder;
    return 0;
}

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
    

    // if(is_use_jps){
    //     node_vis.color.a = 1.0;
    //     node_vis.color.r = 1.0;
    //     node_vis.color.g = 0.0;
    //     node_vis.color.b = 0.0;
    // }
    // else{
    //     node_vis.color.a = 1.0;
    //     node_vis.color.r = 1.0;
    //     node_vis.color.g = 1.0;
    //     node_vis.color.b = 1.0;
    // }


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
        case 1:
        {
            _grid_path_vis_pub_jps.publish(node_vis);
            break;
        }
        case 2:
        {
            _grid_path_vis_pub_rrt.publish(node_vis);
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

        case 1://for JPS
        {
            _visited_nodes_jps_vis_pub.publish(node_vis);
            break;
        }
        
        case 2://for RRT
        {
            _visited_nodes_rrt_vis_pub.publish(node_vis);
            break;
        }
    }
}






void vis_node_line_Path(vector<Vector3d> nodes)
{
    visualization_msgs::Marker Points, Line; 
    Points.header.frame_id = Line.header.frame_id = "world";
    Points.header.stamp    = Line.header.stamp    = ros::Time::now();
    Points.ns              = Line.ns              = "demo_node/RRTstarPath";
    Points.action          = Line.action          = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id   = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = _resolution/2*4; 
    Points.scale.y = _resolution/2*4;
    Line.scale.x   = _resolution/2;

    //points are green and Line Strip is blue
    Points.color.r = 1.0f;
    Points.color.a = 1.0;
    Line.color.b   = 1.0;
    Line.color.a   = 1.0;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _grid_path_vis_pub_rrt.publish(Points);
    _grid_path_vis_pub_rrt.publish(Line); 
}