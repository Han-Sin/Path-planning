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


Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// useful global variables
bool _has_map   = false;

// ros related
ros::Subscriber vel_sub,pos_sub;
ros::Publisher  drone_pos_pub,drone2_pos_pub;
void visVisitedNode( vector<Vector3d> nodes ,int flag);

bool pos_init_flag=1;
Vector3d current_pos;

void Drone_control();



#define D2R (3.14/180.0) //角度制转弧度制

double x_target=0;//目标世界坐标系的x坐标
double y_target=0;//目标世界坐标系的y坐标
double theta_target=0;//目标与世界坐标系x轴的夹角

double x_UAV=1;//飞机世界坐标系的x坐标
double y_UAV=1;//飞机世界坐标系的y坐标
double theta_UAV=0;//飞机与世界坐标系x轴的夹角

double h=5;//飞行定高
double angle_min=20;//保持检测的最小角度
double angle_max=30;//保持检测的最大角度   //实际中用70,此处为调试方便写30
double d_min=tan(angle_min*D2R)*h;  //保持检测的最小距离
double d_max=tan(angle_max*D2R)*h;  //保持检测的最大距离

double v_max=2;  //飞机的速度上限

double x_target_last=x_target;    //目标上一采样周期的x坐标，世界坐标系
double y_target_last=y_target;    //目标上一采样周期的y坐标，世界坐标系


ros::Publisher turtle_vel;

using namespace Eigen;
using namespace std;



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
            visVisitedNode(drone_pos,1);
            x_target=current_pos[0];
            y_target=current_pos[1];
            ros::Rate rate(100);
            rate.sleep();
        }
}







void rcvPosCallBack(visualization_msgs::Marker pos)
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

        for (int i=0;i<pos.points.size();i++)
        {
            // double t_frequency=100;
            // double t_gap=1/t_frequency;
            // double v_x=vel.poses[i].pose.position.x;
            // double v_y=vel.poses[i].pose.position.y;
            // double v_z=vel.poses[i].pose.position.z;
            // double v_mod=sqrt(v_x*v_x+v_y*v_y+v_z*v_z);

            // current_pos[0]+=v_x*t_gap;
            // current_pos[1]+=v_y*t_gap;
            // current_pos[2]+=v_z*t_gap;
            auto coord=pos.points[i];
            current_pos[0]=coord.x;
            current_pos[1]=coord.y;
            current_pos[2]=coord.z;
            
            vector<Vector3d> drone_pos;
            drone_pos.push_back(current_pos);
            visVisitedNode(drone_pos,1);
            ros::Rate rate(100);
            rate.sleep();
        }
}



//输入速度，返回速度的模
double v_mod(double v_x,double v_y)
{
    return sqrt(v_x*v_x+v_y*v_y);
}


//返回安全距离
double my_E(double d)
{
    if(d<d_min)
        return d_min;
    else if (d>d_max)
        return d_max;
    else return d;

}

//输入角度，返回旋转矩阵（绕z轴）
Matrix<double, 3, 3> rotate_matrix(double theta)
{
    Matrix<double, 3, 3> A;
    A<<cos(theta),-sin(theta),0,sin(theta),cos(theta),0,0,0,1;
    return A;
}


void Drone_control()
{

    double d;                //目标与飞机的距离
    double d_x;              //目标与飞机的x轴距离，飞机坐标系
    double d_y;              //目标与飞机的y轴距离，飞机坐标系
    double e_theta;          //目标与飞机的连线的夹角，飞机坐标系
    double e_d;              //距离与检测安全距离之差
    double v_target;         //目标线速度
    double v_target_x;       //目标x轴速度，世界坐标系
    double v_target_y;       //目标y轴速度，世界坐标系
    
    double sample_time=0.01;  //采样时间
    double theta_d;          //目标的速度夹角，飞机坐标系
    double v_x_set;          //飞机速度x指令
    double v_y_set;          //飞机速度y指令
    double k1=1;             //速度比例系数
    double k2=0.2;           //角度比例系数
    Matrix<double, 3, 3> R_WA;   //旋转矩阵RWA  W在上，A在下  下同
    Matrix<double, 3, 3> R_WB;
    Matrix<double, 3, 3> R_AB;

    Matrix<double, 3, 1> VB_W;      //目标速度，世界坐标系
    Matrix<double, 3, 1> VB_A;      //目标速度，飞机坐标系         //接口大概放在这里
    Matrix<double, 3, 1> P_W_BORG;  //B的零点，世界坐标系
    Matrix<double, 4, 1> P_W_BORG_temp;
    Matrix<double, 3, 1> P_W_AORG;
    Matrix<double, 4, 1> P_A_BORG;  //B的零点，A坐标系
    Matrix<double, 3, 4> M_temp34;  //一个3×4的临时矩阵（为了分块凑出T矩阵）
    Matrix<double, 4, 4> T_W_B;     //旋转平移矩阵
    Matrix<double, 4, 4> T_W_A;
    Matrix<double, 4, 4> T_A_B;

    geometry_msgs::Twist vel_msg;



    v_target_x=(x_target-x_target_last)/sample_time;
    v_target_y=(y_target-y_target_last)/sample_time;
    v_target=sqrt(pow((x_target-x_target_last)/sample_time,2)+pow((y_target-y_target_last)/sample_time,2));
    VB_W<<v_target_x,v_target_y,0;

    R_WA=rotate_matrix(theta_UAV);
    R_WB=rotate_matrix(theta_target);
    R_AB=R_WA.inverse()*R_WB;

    P_W_BORG<<x_target,y_target,0;
    M_temp34<<R_WB,P_W_BORG;
    T_W_B<<M_temp34,0,0,0,1;//生成TWB

    P_W_AORG<<x_UAV,y_UAV,0;
    M_temp34<<R_WA,P_W_AORG;
    T_W_A<<M_temp34,0,0,0,1;//生成TA

    P_W_BORG_temp<<P_W_BORG,1;
    P_A_BORG=T_W_A.inverse()*P_W_BORG_temp;

    d_x=P_A_BORG(0,0);
    d_y=P_A_BORG(1,0);
    d=sqrt(d_x*d_x+d_y*d_y);

    cout<<"d= "<<d<<"  d_min=  "<<d_min<<"  d_max= "<<d_max<<endl;

    e_d=d-my_E(d);
    cout<<"ed="<<e_d<<endl;

    VB_A=R_WA.inverse()*VB_W;

    if(VB_A(0,0)==0) //避免分母为0的情况
        theta_d=0;
    else
        theta_d=atan2(VB_A(1,0),VB_A(0,0));


    x_target_last=x_target;
    y_target_last=y_target;


    if(d_x==0)      //避免分母为0的情况
        e_theta=0;
    else
        e_theta=atan2(P_A_BORG(1,0),P_A_BORG(0,0));


    vel_msg.angular.z = e_theta+k2*v_target/d*sin(theta_d-e_theta); //设置角速度

    ROS_INFO("angular:e_theta=%.2f v_target=%.2f d=%.2f theta_d=%.2f  theta_UAV=%.2f",e_theta,v_target,d,theta_d,theta_UAV);

    v_x_set=k1*e_d*cos(e_theta)+v_target*cos(theta_d-e_theta)*cos(e_theta); //计算速度x分量
    v_y_set=k1*e_d*sin(e_theta)+v_target*cos(theta_d-e_theta)*sin(e_theta); //计算速度y分量

    double v_UAV_mod=v_mod(v_x_set,v_y_set);
    if (v_mod(v_x_set,v_y_set)>v_max)  //如果速度超限，限幅
    {
        v_x_set*=v_max/v_mod(v_x_set,v_y_set);
        v_y_set*=v_max/v_mod(v_x_set,v_y_set);
    }

    if(e_d==0)//在安全距离内，不进行跟踪，只旋转
        v_x_set=0;
        v_y_set=0;


    vel_msg.linear.x=v_x_set;
    vel_msg.linear.y=v_y_set;

    // y_UAV+=v_x_set*sin(theta_UAV)*sample_time+v_y_set*cos(theta_UAV)*sample_time;
    // x_UAV+=v_y_set*sin(theta_UAV)*sample_time+v_x_set*cos(theta_UAV)*sample_time;

    x_UAV+=v_x_set*sample_time;
    y_UAV+=v_y_set*sample_time;
    theta_UAV+=vel_msg.angular.z*sample_time;
    ROS_INFO("X_UAV=%f   y_UAV=%f",v_x_set,v_y_set);

    Vector3d dronepos2;
    vector<Vector3d> drone_pos;
    dronepos2<<x_UAV,y_UAV,0;
    drone_pos.push_back(dronepos2);
    visVisitedNode(drone_pos,2);

    // turtle_vel.publish(vel_msg);  //输出速度消息

    // ROS_INFO("UAV : v_x=%.2f  v_y=%.2f ",vel_msg.linear.x, vel_msg.linear.y);
    // ROS_INFO(" ");

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


    vel_sub  = nh.subscribe( "/trajectory_generator_node/vel",       1, rcvVelCallBack );//注释则关闭飞机运动
    // pos_sub  = nh.subscribe( "/trajectory_generator_node/vis_trajectory_besier",       1, rcvPosCallBack );//注释则关闭飞机运动
    drone_pos_pub     = nh.advertise<visualization_msgs::Marker>("drone_pos",50);
    drone2_pos_pub    = nh.advertise<visualization_msgs::Marker>("drone2_pos",50);
    ros::Rate rate(200);
    bool status = ros::ok();

    // ros::AsyncSpinner spinner(4); // Use 4 threads
    
    while(status) 
    {
        ros::spinOnce();      
        Drone_control();
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

    if(flag==1)
        drone_pos_pub.publish(node_vis);
    else if(flag==2)
        drone2_pos_pub.publish(node_vis);
}