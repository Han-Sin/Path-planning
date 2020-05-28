#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <Eigen/Dense>

#define D2R (3.14/180.0) //角度制转弧度制

double x_target=0;//目标世界坐标系的x坐标
double y_target=0;//目标世界坐标系的y坐标
double theta_target=0;//目标与世界坐标系x轴的夹角

double x_UAV=0;//飞机世界坐标系的x坐标
double y_UAV=0;//飞机世界坐标系的y坐标
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


//更新目标位置
void poseCallback1(const turtlesim::PoseConstPtr& msg)
{
    x_target=msg->x;
    y_target=msg->y;
    theta_target=msg->theta;
//    ROS_INFO("turtle1 : x=%.2f  y=%.2f  w=%.2f ",msg->x, msg->y,msg->theta);
}

//更新飞机位置
void poseCallback2(const turtlesim::PoseConstPtr& msg)
{
    x_UAV=msg->x;
    y_UAV=msg->y;
    theta_UAV=msg->theta;
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
    
    double sample_time=0.1;  //采样时间
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

    // cout<<"d= "<<d<<"  d_min=  "<<d_min<<"  d_max= "<<d_max<<endl;

    e_d=d-my_E(d);
    // cout<<"ed="<<e_d<<endl;

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

    // ROS_INFO("angular:e_theta=%.2f v_target=%.2f d=%.2f theta_d=%.2f  theta_UAV=%.2f",e_theta,v_target,d,theta_d,theta_UAV);

    v_x_set=k1*e_d*cos(e_theta)+v_target*cos(theta_d-e_theta)*cos(e_theta); //计算速度x分量
    v_y_set=k1*e_d*sin(e_theta)+v_target*cos(theta_d-e_theta)*sin(e_theta); //计算速度y分量

    double v_UAV_mod=v_mod(v_x_set,v_y_set);
    if (v_mod(v_x_set,v_y_set)>v_max)  //如果速度超限，限幅
    {
        v_x_set*=v_max/v_mod(v_x_set,v_y_set);
        v_y_set*=v_max/v_mod(v_x_set,v_y_set);
    }

    if(e_d==0)//在安全距离内，不进行跟踪，只旋转
    {
        v_x_set=0;
        v_y_set=0;
    }



    vel_msg.linear.x=v_x_set;
    vel_msg.linear.y=v_y_set;

    turtle_vel.publish(vel_msg);  //输出速度消息

    ROS_INFO("UAV : v_x=%.2f  v_y=%.2f ",vel_msg.linear.x, vel_msg.linear.y);
    // ROS_INFO(" ");

}


int main(int argc, char** argv)
{
	// 初始化ROS节点
	ros::init(argc, argv, "my_tf_listener");

    // 创建节点句柄
	ros::NodeHandle node;

	// 请求产生turtle2d
	ros::service::waitForService("/spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	srv.request.x=6;//turtle2d的横坐标
	srv.request.y=6;//turtle2d的纵坐标
	add_turtle.call(srv);

	// 创建发布turtle2速度控制指令的发布者
	turtle_vel = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

	//监听目标姿态消息          //接受信息接口修改处
    ros::Subscriber sub1 = node.subscribe("turtle1/pose", 10, &poseCallback1);

    //监听飞机姿态信息
    ros::Subscriber sub2 = node.subscribe("turtle2/pose", 10, &poseCallback2);

    //速度指令

    //刷新频率10Hz
    ros::Rate rate(10.0);

    
    while(node.ok())
    {
        ros::spinOnce();
        Drone_control();
        rate.sleep();
    }

	return 0;
};
