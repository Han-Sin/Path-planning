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

// #include <waypoint_trajectory_generator/trajpoint.h>

using namespace std;
using namespace Eigen;


// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    


Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// useful global variables
bool _has_map   = false;

// ros related
ros::Subscriber vel_sub;
ros::Publisher  drone_pos_pub;
void visVisitedNode( vector<Vector3d> nodes );

bool pos_init_flag=1;
Vector3d current_pos;

void rcvVelCallBack(nav_msgs::Path vel)
{
        // vector<Vector3d> drone_pos;
        // drone_pos.push_back(_start_pt);
        // ROS_INFO("start_x=%f",drone_pos[0](0));
        // visVisitedNode(drone_pos);

        if(pos_init_flag)
        {
            current_pos=_start_pt;
            pos_init_flag=0;
        }

        for (int i=0;i<vel.poses.size();i++)
        {
            double t_frequency=100;
            double t_gap=1/t_frequency;
            double v_x=vel.poses[i].pose.position.x;
            double v_y=vel.poses[i].pose.position.y;
            double v_z=vel.poses[i].pose.position.z;
            double v_mod=sqrt(v_x*v_x+v_y*v_y+v_z*v_z);

            current_pos[0]+=v_x*t_gap;
            current_pos[1]+=v_y*t_gap;
            current_pos[2]+=v_z*t_gap;
            
            vector<Vector3d> drone_pos;
            drone_pos.push_back(current_pos);
            visVisitedNode(drone_pos);
            ros::Rate rate(100);
            rate.sleep();
        }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_node");
    ros::NodeHandle nh("~");

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


    // vel_sub  = nh.subscribe( "/trajectory_generator_node/vel",       1, rcvVelCallBack );//注释则关闭飞机运动
    drone_pos_pub     = nh.advertise<visualization_msgs::Marker>("drone_pos",50);

    ros::Rate rate(200);
    bool status = ros::ok();

    // ros::AsyncSpinner spinner(4); // Use 4 threads
    
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }
    // spinner.stop();

    return 0;
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
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    // node_vis.scale.x = _resolution;
    // node_vis.scale.y = _resolution;
    // node_vis.scale.z = _resolution;

    node_vis.scale.x = _resolution*2;
    node_vis.scale.y = _resolution*2;
    node_vis.scale.z = _resolution*2;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    drone_pos_pub.publish(node_vis);
}