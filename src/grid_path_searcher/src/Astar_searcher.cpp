#include "Astar_searcher.h"
#include <math.h>
#include <nav_msgs/Path.h>
#include <utility>
using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);
    //-25 -25 0 lower 
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    //25 25 5 upper 
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    //初始化一张高分辨率障碍地图，倍率为2（2×2×2）
    data_high_resolution=new uint8_t[GLXYZ_SIZE*resolution_ratio*resolution_ratio*resolution_ratio];
    memset(data_high_resolution, 0, GLXYZ_SIZE*resolution_ratio*resolution_ratio*resolution_ratio * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);//index
                Vector3d pos = gridIndex2coord(tmpIdx);//coord
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = int( (coord_x - gl_xl) * inv_resolution);
    // ROS_INFO("ORI_X=%f     idx_x=%d   ",( (coord_x - gl_xl) * inv_resolution),idx_x);
    int idx_y = int( (coord_y - gl_yl) * inv_resolution);
    int idx_z = int( (coord_z - gl_zl) * inv_resolution);

    double expand_ratio=1;

    double default_resolution=0.2;
    int expand_size=(int)(expand_ratio*(double)(default_resolution/resolution));//膨胀栅格数，0时不膨胀，1够用
    if(expand_size<=0)
        expand_size=1;

    for (int i=-expand_size;i<=expand_size;i++)
        for (int j=-expand_size;j<=expand_size;j++)
            for (int k=-expand_size;k<=expand_size;k++)
            {
                int temp_x=idx_x+i;
                int temp_y=idx_y+j;
                int temp_z=idx_z+k;

                double rev_x=(double)temp_x/inv_resolution+gl_xl;
                double rev_y=(double)temp_y/inv_resolution+gl_yl;
                double rev_z=(double)temp_z/inv_resolution+gl_zl;

                if( rev_x < gl_xl  || rev_y < gl_yl  || rev_z <  gl_zl ||
                    rev_x >= gl_xu || rev_y >= gl_yu || rev_z >= gl_zu )
                    continue;
//                ROS_WARN("expand suc,%d  %d  %d  ",i,j,k);
                data[temp_x * GLYZ_SIZE + temp_y * GLZ_SIZE + temp_z] = 1;//index(grid)
            }
    

    //高分辨率地图的创建
    double high_resolution=resolution/resolution_ratio;//分辨率
    double high_inv_resolution=1/high_resolution;//反分辨率
    //高分辨率障碍物地图设置
    idx_x = int( (coord_x - gl_xl) * high_inv_resolution);
    // ROS_INFO("ORI_X=%f     idx_x=%d   ",( (coord_x - gl_xl) * inv_resolution),idx_x);
    idx_y = int( (coord_y - gl_yl) * high_inv_resolution);
    idx_z = int( (coord_z - gl_zl) * high_inv_resolution);
    
    int high_expand_size=(int)(expand_ratio*(double)(default_resolution/high_resolution));//膨胀单位
    // ROS_WARN("expand_size=%d    high_expand_size=%d  ",expand_size,high_expand_size);
    for (int i=-high_expand_size;i<=high_expand_size;i++)
        for (int j=-high_expand_size;j<=high_expand_size;j++)
            for (int k=-high_expand_size;k<=high_expand_size;k++)
            {
                int temp_x=idx_x+i;
                int temp_y=idx_y+j;
                int temp_z=idx_z+k;

                double rev_x=(double)temp_x/high_inv_resolution+gl_xl;
                double rev_y=(double)temp_y/high_inv_resolution+gl_yl;
                double rev_z=(double)temp_z/high_inv_resolution+gl_zl;

                if( rev_x < gl_xl  || rev_y < gl_yl  || rev_z <  gl_zl ||
                    rev_x >= gl_xu || rev_y >= gl_yu || rev_z >= gl_zu )
                    continue;
//                ROS_WARN("expand suc,%d  %d  %d  ",i,j,k);
                //障碍物的下标设置需要注意，xyz分别要乘以分辨率倍率的平方，一次，零次
                data_high_resolution[temp_x * GLYZ_SIZE*resolution_ratio*resolution_ratio + temp_y * GLZ_SIZE *resolution_ratio+ temp_z ] = 1;//index(grid)
            }

}


bool AstarPathFinder::Nodes_if_in_Path(vector<Vector3d> path,Vector3d node)
{
    int i;
    for (i=0;i<path.size();i++)
    {
        if (path[i]==node)
            return false;
    }
    return true;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{

    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr tempPtr=terminatePtr;    

    // while(terminatePtr!=NULL){
    //     gridPath.push_back(terminatePtr);
    //     terminatePtr = terminatePtr->cameFrom;
    // }
    // for (auto ptr: gridPath)
    //     path.push_back(ptr->coord);

    while(tempPtr!=NULL){
        gridPath.push_back(tempPtr);
        tempPtr = tempPtr->cameFrom;
    }
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);


    reverse(path.begin(),path.end());

    ROS_WARN("path_nodes size : %d", path.size());


    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1 && Nodes_if_in_Path(path,GridNodeMap[i][j][k]->coord))  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index)
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt)
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}
inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    int  x = currentPtr->index(0);
    int  y = currentPtr->index(1);
    int  z = currentPtr->index(2);
    for(int dir_x =-1;dir_x<=1;dir_x++){
        for(int dir_y=-1;dir_y<=1;dir_y++){
            for(int dir_z=-1;dir_z<=1;dir_z++){
                if(dir_x!=0||dir_y!=0||dir_z!=0){{
                    int neighbor_x = x+dir_x;
                    int neighbor_y = y+dir_y;
                    int neighbor_z = z+dir_z;
                    if(isFree(neighbor_x,neighbor_y,neighbor_z)){
                        neighborPtrSets.push_back(GridNodeMap[neighbor_x][neighbor_y][neighbor_z]);
                    }
                }

                }
            }
        }
    }

    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself
    please write your code below
    *
    *
    */
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /*
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    int  x1 = node1->index(0);
    int  y1 = node1->index(1);
    int  z1 = node1->index(2);
    int  x2 = node2->index(0);
    int  y2 = node2->index(1);
    int  z2 = node2->index(2);
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2));

}

double AstarPathFinder::getG(GridNodePtr node1, GridNodePtr node2)
{
    /*
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    //return 1;
    int  x1 = node1->index(0);
    int  y1 = node1->index(1);
    int  z1 = node1->index(2);
    int  x2 = node2->index(0);
    int  y2 = node2->index(1);
    int  z2 = node2->index(2);

    int diff=abs(x1-x2)+abs(y1-y2)+abs(z1-z2);
    if(diff==3)
        return 1.73;
    else if (diff==2)
        return 1.41;
    else if (diff==1)
        return 1;
    else
        ROS_WARN("diff=%f ",diff);
        return 1;

}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; //1 openlist,-1 closelist,0 unvisted
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    //!!!
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->id = 1;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->gScore = 0;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->fScore = startPtr->fScore;

    //GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code beow
    *
    *
    remove from openlist, and add to close list.
    */

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below

        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        // if the current node is the goal
        //缺少判断语句
        std::multimap<double, GridNodePtr> ::iterator it;
        it = openSet.begin();
        currentPtr = (*it).second;
        openSet.erase(it);
        currentPtr->id = -1;
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *
        */
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below

            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : unexpanded
            neighborPtrSets[i]->id = 1 : expanded, equal to this node is in close set
            *
            */
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *new node
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                /*startPtr -> gScore = 0;
                startPtr -> fScore = getHeu(startPtr,endPtr);
                //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
                startPtr -> id = 1; //1 openlist,-1 closelist,0 unvisted
                startPtr -> coord = start_pt;
                openSet.insert( make_pair(startPtr -> fScore, startPtr) );*/
                // neighborPtr->gScore=currentPtr->gScore+1;
                neighborPtr->gScore=currentPtr->gScore+getG(neighborPtr,currentPtr);
                neighborPtr->fScore=neighborPtr->gScore+getHeu(neighborPtr,endPtr);
                neighborPtr->cameFrom=currentPtr;
                neighborPtr->id=1;//push into openlist
                openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
                continue;
            }
            else if(neighborPtr->id==1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                if(neighborPtr->gScore>currentPtr->gScore+getG(neighborPtr,currentPtr)){
                    //change
                    // neighborPtr->gScore = currentPtr->gScore+1;
                    neighborPtr->gScore=currentPtr->gScore+getG(neighborPtr,currentPtr);
                    neighborPtr->fScore = neighborPtr->gScore+getHeu(neighborPtr,endPtr);
                    neighborPtr->cameFrom = currentPtr;
                }
                continue;
            }
            else if(neighborPtr->id==-1){//this node is in closed set
                /*
                *
                please write your code below
                *
                */
                if(neighborPtr->gScore>currentPtr->gScore+getG(neighborPtr,currentPtr)){
                    // neighborPtr->gScore = currentPtr->gScore+1;
                    neighborPtr->gScore=currentPtr->gScore+getG(neighborPtr,currentPtr);
                    neighborPtr->fScore = neighborPtr->gScore+getHeu(neighborPtr,endPtr);
                    neighborPtr->cameFrom = currentPtr;
                    neighborPtr->id=1;
                    openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
                }
                continue;
            }
        }
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr tempPtr=terminatePtr;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *
    */
    while(terminatePtr!=NULL){
        gridPath.push_back(terminatePtr);
        terminatePtr = terminatePtr->cameFrom;
    }
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    terminatePtr=tempPtr;

    return path;
}

//第一版，只拉直，已弃用
vector<Vector3d> AstarPathFinder::getTurningPoints()
// void AstarPathFinder::getTurningPoints()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr currentPtr=terminatePtr;

    GridNodePtr nextPtr=terminatePtr;

    GridNodePtr lastPtr;

    GridNodePtr lastTurningPtr=terminatePtr;

    gridPath.push_back(lastTurningPtr);

    double K_xy=0;
    double K_xy_current;
    double K_yz=0;
    double K_yz_current;
    double my_inf=99999;
    int K_init_flag=1;

    // ROS_INFO("x=%d   y=%d    ",currentPtr->index(0),currentPtr->index(1));
    while(currentPtr->cameFrom!=NULL)
    {
        lastPtr=currentPtr->cameFrom;   

        int  x1 = lastTurningPtr->index(0);
        int  y1 = lastTurningPtr->index(1);
        int  z1 = lastTurningPtr->index(2);
        int  x2 = lastPtr->index(0);
        int  y2 = lastPtr->index(1);
        int  z2 = lastPtr->index(2);

        if(x1==x2)
            K_xy_current=my_inf;
        else
            K_xy_current=double(y1-y2)/(x1-x2);

        if(z1==z2)
            K_yz_current=my_inf;
        else
            K_yz_current=double(y1-y2)/(z1-z2);

        // ROS_INFO("K_current=%f  K=%f init_flag=%d x1=%d   y1=%d    x2=%d    y2=%d",K_xy_current,K_xy,K_init_flag,x1,y1,x2,y2);
              
        if(K_init_flag)
        {
            K_xy=K_xy_current;
            K_yz=K_yz_current;
            K_init_flag=0;
        }

        if (K_xy_current!=K_xy||K_yz_current!=K_yz)
        {
            K_init_flag=1;
            lastTurningPtr=currentPtr;
            gridPath.push_back(lastTurningPtr);
            continue;
        }

        nextPtr=currentPtr;
        currentPtr=lastPtr;
        // break;
    }
    gridPath.push_back(currentPtr);

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());
    return path;

}


//输入A星得到的路径点，输出简化后的关键点
pair<vector<Vector3d>,nav_msgs::Path> AstarPathFinder::getSimplifiedPoints()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr currentPtr=terminatePtr;//从终点开始往前找

    // GridNodePtr nextPtr=terminatePtr;//current的下一个节点（终点方向）(常规意义上的上一个节点)   
    //在这个函数里没啥用，不过还是维护了

    GridNodePtr lastPtr;//current的上一个节点（起点方向）（常规意义上的下一个节点）

    GridNodePtr lastTurningPtr=terminatePtr;//上一个关键点

    gridPath.push_back(lastTurningPtr);//终点肯定是关键点

    while(currentPtr->cameFrom!=NULL)
    {
        GridNodePtr maxPtr=currentPtr->cameFrom;//初始化最远无碰节点
        while(currentPtr->cameFrom!=NULL)
        {
            lastPtr=currentPtr->cameFrom;   

            int  x1 = lastTurningPtr->index(0);
            int  y1 = lastTurningPtr->index(1);
            int  z1 = lastTurningPtr->index(2);
            int  x2 = lastPtr->index(0);
            int  y2 = lastPtr->index(1);
            int  z2 = lastPtr->index(2);

            int collision_flag=0;//碰撞标志位

            double divide_piece_num=20;//碰撞检测划分份数
            for (double k=0;k<1;k+=1.0/divide_piece_num)
            {
                //得到等分点坐标
                int x_check=int(x1*resolution_ratio+(double)k*(x2-x1)*resolution_ratio);
                int y_check=int(y1*resolution_ratio+(double)k*(y2-y1)*resolution_ratio);
                int z_check=int(z1*resolution_ratio+(double)k*(z2-z1)*resolution_ratio);
                // ROS_INFO("check_x=%d   y=%d   z=%d",x_check,y_check,z_check);
                if(if_collision(x_check,y_check,z_check))
                    collision_flag=1;
            }
            if(collision_flag==0)
            {
                // ROS_INFO("collision!");
                //解决死循环
                // if(lastTurningPtr==currentPtr)
                //     {
                //         ROS_WARN("dead loop occurs!!!");
                //         nextPtr=currentPtr;
                //         currentPtr=lastPtr;
                //         continue;
                //     }
                maxPtr=lastPtr;//若无碰，更新最大无碰节点
            }

            // nextPtr=currentPtr;//更新next
            currentPtr=lastPtr;//更新current

        }
        lastTurningPtr=maxPtr;//更新最新的关键点
        gridPath.push_back(lastTurningPtr);//储存关键点
        currentPtr=maxPtr;//更新current

        // break;
    }
    // gridPath.push_back(currentPtr);
    
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    for (auto ptr: gridPath)
    {
        path.push_back(ptr->coord);//维护path
        ROS_INFO("coord_x=%f   y=%f   z=%f",ptr->coord(0),ptr->coord(1),ptr->coord(2));
        pt.pose.position.y =  ptr->coord(1);
        pt.pose.position.x =  ptr->coord(0);
        pt.pose.position.z =  ptr->coord(2);
        waypoints.poses.push_back(pt);//维护waypoints
    }
        
    reverse(path.begin(),path.end());//这步在可视化上没有区别

    pair<vector<Vector3d>,nav_msgs::Path> p;
    p=make_pair(path,waypoints);
    return p;//返回path和waypoints

}


bool AstarPathFinder::if_collision(int x,int y,int z)
{
    return data_high_resolution[x * GLYZ_SIZE*resolution_ratio*resolution_ratio + y * GLZ_SIZE *resolution_ratio+ z ];
}
