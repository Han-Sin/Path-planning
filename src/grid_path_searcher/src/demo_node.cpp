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

#include "graph_searcher.h"
#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
//#include <waypoint_trajectory_generator/trajpoint.h>
//#include <waypoint_trajectory_generator/Trajectoy.h>
// #include <waypoint_trajectory_generator/trajpoint.h>

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define use_RRT 1

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub,_traj_sub,drone_pos_sub;
ros::Publisher  _grid_path_vis_pub,_grid_path_vis_pub_jps, _visited_nodes_vis_pub,
_visited_nodes_jps_vis_pub, _grid_map_vis_pub,_simplified_waypoints_pub,_RRTstar_path_vis_pub;
AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
JPSPathFinder   * _jps_path_finder       = new JPSPathFinder();

RRTstarPreparatory * _RRTstar_preparatory     = new RRTstarPreparatory();

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );
void visVisitedNode( vector<Vector3d> nodes,bool flag);
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

void pathFinding_RRT(const Vector3d start_pt, const Vector3d target_pt);
void visRRTstarPath(vector<Vector3d> nodes );


// Our collision checker. For this demo, our robot's state space
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {   
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state3D =
            state->as<ob::RealVectorStateSpace::StateType>();
        /**
        *
        *
        STEP 1: Extract the robot's (x,y,z) position from its state
        *
        *
        */
        double x = state3D->values[0];
        double y = state3D->values[1];
        double z = state3D->values[2];
        return _RRTstar_preparatory->isObsFree(x, y, z);
    }
};

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

void pathFinding_RRT(const Vector3d start_pt, const Vector3d target_pt)
{
    // Construct the robot state space in which we're planning. 
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));

    // Set the bounds of space to be in [0,1].
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, - _x_size * 0.5);
    bounds.setLow(1, - _y_size * 0.5);
    bounds.setLow(2, 0.0);

    bounds.setHigh(0, + _x_size * 0.5);
    bounds.setHigh(1, + _y_size * 0.5);
    bounds.setHigh(2, _z_size);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();

    // Set our robot's starting state
    ob::ScopedState<> start(space);
    start[0] = start_pt(0);
    start[1] = start_pt(1);
    start[2] = start_pt(2);

    // Set our robot's goal state
    ob::ScopedState<> goal(space);


    goal[0] = target_pt(0);
    goal[1] = target_pt(1);
    goal[2] = target_pt(2);    


   auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    pdef->setOptimizationObjective(getPathLengthObjective(si));
    // Set the problem instance for our planner to solve
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem within one second of
    // planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
        
        vector<Vector3d> path_points;

        for (size_t path_idx = 0; path_idx < path->getStateCount (); path_idx++)
        {
            const ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>(); 
            Vector3d point;
            point(0) = (*state)[0];
            point(1) = (*state)[1];
            point(2) = (*state)[2];
            path_points.push_back(point);            

        }
        visGridPath(path_points,1);  
        // visRRTstarPath(path_points);     
    }
}


void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    if (!use_RRT)
        pathFinding(_start_pt, target_pt); 
    else
        pathFinding_RRT(_start_pt, target_pt); 
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
        _RRTstar_preparatory->setObs(pt.x, pt.y, pt.z);

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

    ros::Time time_1 = ros::Time::now();

#define _use_jps 1
#if _use_jps
{
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
    //Retrieve the path
    auto grid_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();
    auto simplified_points_path = _astar_path_finder->getSimplifiedPoints(1000);//化简后的关键点（100指不中间采样）
    // ROS_INFO("Using A*!");
    //发布不采样的关键点
    // auto simplified_points_path2 = _astar_path_finder->getSimplifiedPoints(100);//化简后的关键点(不采样)
    // _simplified_waypoints_pub2.publish(_astar_path_finder->vector3d_to_waypoints(simplified_points_path2));
    // auto simplified_points_path = _astar_path_finder->getSimplifiedPoints_by_lines();//化简后的关键点,直线查找
    nav_msgs::Path simplified_waypoints=_astar_path_finder->vector3d_to_waypoints(simplified_points_path);
    // nav_msgs::Path simplified_waypoints=_astar_path_finder->vector3d_to_waypoints(grid_path);
    _simplified_waypoints_pub.publish(simplified_waypoints);//发布关键点
    // _simplified_waypoints_pub3.publish(_astar_path_finder->vector3d_to_waypoints(temp_path));
    // visVisitedNode(simplified_points_path);//可视化关键点
    // _simplified_waypoints_pub.publish(simplified_waypoints);
    // _simplified_waypoints_pub.publish(_astar_path_finder->vector3d_to_waypoints(simplified_path_RDP));
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("Total time cost is %f ms", (time_2 - time_1).toSec() * 1000.0);
    //Visualize the result
    visGridPath (grid_path, false);//可视化搜寻的栅格地图
    visVisitedNode(visited_nodes,0);
    // visVisitedNode(simplified_points_path);
    // visVisitedNode(simplified_path_RDP);
    //Reset map for next call
    _astar_path_finder->resetUsedGrids();
    //_use_jps = 0 -> Do not use JPS
    //_use_jps = 1 -> Use JPS
    //you just need to change the #define value of _use_jps
}
#endif
    
#if _use_jps
    {
        // ROS_INFO("Using JPS!");
        //Call JPS to search for a path
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, _use_jps);
        visVisitedNode(visited_nodes,1);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
    }
#endif
}

void rcvDronePosCallBack(const visualization_msgs::Marker &pos_msg)
{
    // ROS_INFO("rcv pos!");
    // ROS_INFO("%f",pos_msg.points[0].x);

    Vector3d current_pt(pos_msg.points[0].x,pos_msg.points[0].y,pos_msg.points[0].z);
    _start_pt=current_pt;

}

int main(int argc, char** argv)
{
    cout<<int(5.5)<<"   "<<int(-5.5)<<"   "<<int(5.6)<<"    "<<int(-5.6)<<endl;
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    drone_pos_sub = nh.subscribe("/drone_node/drone_pos",50,rcvDronePosCallBack);

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _grid_path_vis_pub_jps        = nh.advertise<visualization_msgs::Marker>("grid_path_vis_jps", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
    _visited_nodes_jps_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_jps_vis", 1);
    _simplified_waypoints_pub     = nh.advertise<nav_msgs::Path>("simplified_waypoints",50);//发布优化后的轨迹
    
    _RRTstar_path_vis_pub         = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis",1);

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
    
    _RRTstar_preparatory  = new RRTstarPreparatory();
    _RRTstar_preparatory  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
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
    }
    // spinner.stop();

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

    if(is_use_jps)
        _grid_path_vis_pub_jps.publish(node_vis);
    else
        _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode(vector<Vector3d> nodes,bool is_use_jps)
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

    if(is_use_jps)
    {
        node_vis.color.a = 0.4;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 1.0;
    }
    else
    {
        node_vis.color.a = 0.15;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
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

    if(is_use_jps)
        _visited_nodes_jps_vis_pub.publish(node_vis);
    else
        _visited_nodes_vis_pub.publish(node_vis);
}

void visRRTstarPath(vector<Vector3d> nodes )
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

    Points.scale.x = _resolution/2; 
    Points.scale.y = _resolution/2;
    Line.scale.x   = _resolution/2;

    //points are green and Line Strip is blue
    Points.color.g = 1.0f;
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
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line); 
}