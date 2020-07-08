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

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>

// #include <waypoint_trajectory_generator/trajpoint.h>

using namespace std;
using namespace Eigen;


// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    
int frequency  = 1000;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// useful global variables
bool _has_map   = false;

// ros related
ros::Subscriber vel_sub,vel_sub2,front_pos_sub,acc_sub,acc_sub2,jerk_sub,hybridAstar_vel_sub;
ros::Publisher  drone_pos_pub,drone2_pos_pub,v_a_pub,front_v_a_pub;
void visVisitedNode( vector<Vector3d> nodes ,int flag);
void vis_front_drone(Vector3d pos);
bool pos_init_flag=1;
Vector3d current_pos;
Vector3d back_drone_pos(0,0,2);
double x_target=0;//目标世界坐标系的x标
double y_target=0;//目标世界坐标系的y坐标
double x_target_last=x_target;    //目标上一采样周期的x坐标，世界坐标系
double y_target_last=y_target;    //目标上一采样周期的y坐标，世界坐标系



using namespace Eigen;
using namespace std;

nav_msgs::Path _vel;
nav_msgs::Path _vel2;
nav_msgs::Path _acc;
nav_msgs::Path _acc2;
nav_msgs::Path _jerk;
nav_msgs::Path _front_pos;
int update_vel_flag=0;//路径更新标志位
int begin_control_flag=0;//控制飞机标志位
int update_frontpos_flag=0;
int begin_frontpos=0;
int update_vel_flag2=0;//路径更新标志位
int begin_control_flag2=0;//控制飞机标志位



void rcvJerkCallBack(nav_msgs::Path jerk)
{
        _jerk=jerk;
        ROS_WARN("jerk update!!!");
}

void rcvAccCallBack(nav_msgs::Path acc)
{
        _acc=acc;
        // ROS_WARN("acc update!!!");
}

void rcvVelCallBack(nav_msgs::Path vel)
{
        _vel=vel;
        update_vel_flag=1;
        ROS_WARN("Begin to run!");
}

void rcvVelCallBack2(nav_msgs::Path vel)
{
        _vel2=vel;
        update_vel_flag2=1;
}

void rcvFrontPosCallBack(nav_msgs::Path pos)
{
    _front_pos = pos;
    update_frontpos_flag=1;
}

double last_v_x_front=0;
double last_v_y_front=0;
double last_v_z_front=0;

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

void Front_Drone_Control(int &i)
{
    // vector<Vector3d> drone_pos;
        // drone_pos.push_back(_start_pt);
        // ROS_INFO("start_x=%f",drone_pos[0](0));
        // visVisitedNode(drone_pos);
        //ROS_WARN("111111111111111111111111111!");
        if(pos_init_flag)
        {
            current_pos=_start_pt;
            pos_init_flag=0;
        }
        //ROS_WARN("2222222222222222222222");
        if(update_vel_flag==1)
        {
            update_vel_flag=0;
            i=0;
        }
        //ROS_WARN("333333333333333333");
        double t_frequency=frequency;
        double t_gap=1/t_frequency;
        double v_x=_vel.poses[i].pose.position.x;
        double v_y=_vel.poses[i].pose.position.y;
        double v_z=_vel.poses[i].pose.position.z;
        double v_mod=sqrt(v_x*v_x+v_y*v_y+v_z*v_z);
        //ROS_WARN("4444444444444444");
        // cout<<"vx: "<<v_x<<" vy: "<<v_y<<" vz: "<<v_z<<endl;
        current_pos[0]+=v_x*t_gap;
        current_pos[1]+=v_y*t_gap;
        current_pos[2]+=v_z*t_gap;
        last_v_x_front=v_x;
        last_v_y_front=v_y;
        last_v_z_front=v_z;
        vis_front_drone(current_pos);
        //ROS_INFO_STREAM("current_pos: "<<current_pos);
        /*vector<Vector3d> drone_pos;
        drone_pos.push_back(current_pos); 
        visVisitedNode(drone_pos,1);
        */
        i++;
        if(i>=_vel.poses.size())
        {
            begin_control_flag=0;
            i=0;
        }
        
        /*        
        // ROS_INFO("1!!!!");
        double a_x=_acc.poses[i].pose.position.x;
        double a_y=_acc.poses[i].pose.position.y;
        double a_z=_acc.poses[i].pose.position.z;

        // double j_x=(v_x-last_v_x_front)/t_gap;
        // double j_y=(v_y-last_v_y_front)/t_gap;
        // double j_z=(v_z-last_v_z_front)/t_gap;

        double j_x=_jerk.poses[i].pose.position.x;
        double j_y=_jerk.poses[i].pose.position.y;
        double j_z=_jerk.poses[i].pose.position.z;

        // ROS_INFO("2!!!");
        last_v_x_front=v_x;
        last_v_y_front=v_y;
        last_v_z_front=v_z;

        vector<Vector3d> v_a_msg;
        Vector3d v_msg(v_x,v_y,v_z);
        Vector3d a_msg(a_x,a_y,a_z);
        Vector3d j_msg(j_x,j_y,j_z);
        v_a_msg.push_back(v_msg);
        v_a_msg.push_back(a_msg);
        v_a_msg.push_back(j_msg);
        ROS_WARN("55555555555555");
        front_v_a_pub.publish(vector3d_to_waypoints(v_a_msg));
        vector<Vector3d> drone_pos;
        ROS_WARN("666666666666666");
        drone_pos.push_back(current_pos);
        visVisitedNode(drone_pos,1);
        x_target=current_pos[0];
        y_target=current_pos[1];
        //ROS

        // Back_Drone_control();

        // ros::Rate rate(100);
        // rate.sleep();

        i++;
        if(i>=_vel.poses.size())
        {
            begin_control_flag=0;
            i=0;
        }*/

}

void Front_Drone_Pos(int &k){
    if(pos_init_flag){
        current_pos=_start_pt;
        pos_init_flag=0;
    }
    if(update_frontpos_flag==1)
    {
        update_frontpos_flag=0;
        k=0;
    }
    double pos_x=_front_pos.poses[k].pose.position.x;
    double pos_y=_front_pos.poses[k].pose.position.y;
    double pos_z=_front_pos.poses[k].pose.position.z;
    current_pos[0]=pos_x;
    current_pos[1]=pos_y;
    current_pos[2]=pos_z;
    k++;
    if(k>=_front_pos.poses.size())
    {
        begin_control_flag=0;
        k=0;
    }
}


double last_v_x_back=0;
double last_v_y_back=0;
double last_v_z_back=0;

void Back_Drone_Control2(int &i)
{

        if(update_vel_flag2==1)
        {
            update_vel_flag2=0;
            i=0;
        }

        double t_frequency=frequency;
        double t_gap=1/t_frequency;
        double v_x=_vel2.poses[i].pose.position.x;
        double v_y=_vel2.poses[i].pose.position.y;
        double v_z=_vel2.poses[i].pose.position.z;
        double v_mod=sqrt(v_x*v_x+v_y*v_y+v_z*v_z);

        back_drone_pos[0]+=v_x*t_gap;
        back_drone_pos[1]+=v_y*t_gap;
        back_drone_pos[2]+=v_z*t_gap;

        double a_x=(v_x-last_v_x_back)/t_gap;
        double a_y=(v_y-last_v_y_back)/t_gap;
        double a_z=(v_z-last_v_z_back)/t_gap;

        last_v_x_back=v_x;
        last_v_y_back=v_y;
        last_v_z_back=v_z;

        vector<Vector3d> v_a_msg;
        Vector3d v_msg(v_x,v_y,v_z);
        Vector3d a_msg(a_x,a_y,a_z);
        v_a_msg.push_back(v_msg);
        v_a_msg.push_back(a_msg);
        v_a_pub.publish(vector3d_to_waypoints(v_a_msg));
        


        
        vector<Vector3d> drone_pos;
        drone_pos.push_back(back_drone_pos);
        visVisitedNode(drone_pos,2);

        // Back_Drone_control();

        // ros::Rate rate(100);
        // rate.sleep();

        i++;
        // cout<<"i="<<i<<endl;
        if(i>=_vel2.poses.size())
        {
            begin_control_flag2=0;
            i=0;
        }


}







//输入速度，返回速度的模
double v_mod(double v_x,double v_y)
{
    return sqrt(v_x*v_x+v_y*v_y);
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

    hybridAstar_vel_sub = nh.subscribe("/hybridAstar/vel",1,rcvVelCallBack);
    //vel_sub  = nh.subscribe( "/trajectory_generator_node/vel",       1, rcvVelCallBack );//注释则关闭飞机运动
    acc_sub  = nh.subscribe( "/trajectory_generator_node/acc",       1, rcvAccCallBack );
    jerk_sub  = nh.subscribe( "/trajectory_generator_node/jerk",       1, rcvJerkCallBack );
    vel_sub2  = nh.subscribe( "/trajectory_generator_node/vel2",       1, rcvVelCallBack2 );//注释则关闭飞机运动
    // pos_sub  = nh.subscribe( "/trajectory_generator_node/vis_trajectory_besier",       1, rcvPosCallBack );//注释则关闭飞机运动
    front_pos_sub = nh.subscribe("/trajectory_generator_node/front_pos",    1, rcvFrontPosCallBack);
    drone_pos_pub     = nh.advertise<visualization_msgs::Marker>("drone_pos",50);
    drone2_pos_pub    = nh.advertise<visualization_msgs::Marker>("drone2_pos",50);
    v_a_pub           = nh.advertise<nav_msgs::Path>("back_drone_v_a",50);
    front_v_a_pub = nh.advertise<nav_msgs::Path>("front_drone_v_a",50);

    ros::Rate rate(frequency);
    bool status = ros::ok();

    // ros::AsyncSpinner spinner(4); // Use 4 threads
    
    int i=0;
    int j=0;
    int k=0;
    while(status) 
    {
        ros::spinOnce();      
        // Back_Drone_control();
        //if(update_frontpos_flag)
        //    begin_frontpos=1;
        //if(begin_frontpos)
        //    Front_Drone_Pos(k);
        if(update_vel_flag==1)
            begin_control_flag=1;
        if(begin_control_flag==1)
            Front_Drone_Control(i);
        if(update_vel_flag2==1)
            begin_control_flag2=1;
        if(begin_control_flag2==1)
            Back_Drone_Control2(j);
        status = ros::ok();
        rate.sleep();
    }
    // spinner.stop();

    return 0;
}

void visVisitedNode( vector<Vector3d> nodes ,int flag)
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
    if(flag==1)
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else if(flag==2)
    {
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }


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

    if(flag==1)
        drone_pos_pub.publish(node_vis);
    else if(flag==2)
        drone2_pos_pub.publish(node_vis);
}
void vis_front_drone(Vector3d pos){
     //可视化飞机
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.frame_id    = "world";
    _traj_vis.ns = "drone_node/front_pos";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.15;
    _traj_vis.scale.y = 0.15;
    _traj_vis.scale.z = 0.15;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 1.0;
    _traj_vis.color.b = 0.0;
    _traj_vis.points.clear();
    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    _traj_vis.points.push_back(pt);
    drone_pos_pub.publish(_traj_vis);

}