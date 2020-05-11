#include "Astar_searcher.h"
#include <math.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <utility>
using namespace std;
using namespace Eigen;

void TrajectoryCallBack(const visualization_msgs::Marker  & trajectory);

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

    double expand_ratio=0;

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
    
    double expand_scale_ratio=1;//高分辨率地图中，障碍物膨胀稍微小点//事实证明不能小。。。小了会撞
    double expand_ratio_high=0.5;

    int high_expand_size=(int)(expand_ratio_high*expand_scale_ratio*(double)(default_resolution/high_resolution));//膨胀单位
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
vector<Vector3d> AstarPathFinder::getSimplifiedPoints()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr currentPtr=terminatePtr;//从终点开始往前找

    // GridNodePtr nextPtr=terminatePtr;//current的下一个节点（终点方向）(常规意义上的上一个节点)   
    //在这个函数里没啥用，不过还是维护了

    GridNodePtr lastPtr;//current的上一个节点（起点方向）（常规意义上的下一个节点）

    GridNodePtr lastTurningPtr=terminatePtr;//上一个关键点

    gridPath.push_back(lastTurningPtr);//终点肯定是关键点

    int line_point_count=0;
    int temp_count=0;
    while(currentPtr->cameFrom!=NULL)
    {
        GridNodePtr maxPtr=currentPtr->cameFrom;//初始化最远无碰节点
        while(currentPtr->cameFrom!=NULL)
        {
            temp_count++;
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
                maxPtr=lastPtr;//若无碰，更新最大无碰节点
                line_point_count=temp_count;
            }

            // nextPtr=currentPtr;//更新next
            currentPtr=lastPtr;//更新current

        }

        // double default_resolution=0.2;
        // int point_gap_max=3*(int)(default_resolution/resolution);//关键点间最大间隔数

        // if(line_point_count>point_gap_max)//如果直线太长，等分成若干份
        //     {
        //         int divide_num=(int)line_point_count/point_gap_max+1;
        //         int gap=(int)line_point_count/divide_num;
        //         int temp_count=0;
        //         // GridNodePtr tempPtr=lastTurningPtr;
                
        //         int  x_last = lastTurningPtr->index(0);
        //         int  y_last = lastTurningPtr->index(1);
        //         int  z_last = lastTurningPtr->index(2);
        //         int  x_current = maxPtr->index(0);
        //         int  y_current = maxPtr->index(1);
        //         int  z_current = maxPtr->index(2);

        //         for (int i=1;i<divide_num;i++)//i从1开始避免重点
        //         {
        //             Vector3i temp_idx;
        //             temp_idx(0)=(int)(x_last+(double)(x_current-x_last)*(double)i/divide_num);
        //             temp_idx(1)=(int)(y_last+(double)(y_current-y_last)*(double)i/divide_num);
        //             temp_idx(2)=(int)(z_last+(double)(z_current-z_last)*(double)i/divide_num);

        //              GridNodePtr pushPtr = new GridNode(temp_idx, gridIndex2coord(temp_idx));
        //              gridPath.push_back(pushPtr);
        //         }
        //     }    

        lastTurningPtr=maxPtr;//更新最新的关键点
        temp_count=0;
        line_point_count=0;
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
    }
        
    reverse(path.begin(),path.end());//这步在可视化上没有区别

    //1.得到了第一次的关键点path
    return path;

}


// //输入A星得到的路径点，输出简化后的关键点
// vector<Vector3d> AstarPathFinder::getSimplifiedPoints()
// {
//     vector<Vector3d> path;
//     vector<GridNodePtr> gridPath;

//     GridNodePtr currentPtr=terminatePtr;//从终点开始往前找

//     // GridNodePtr nextPtr=terminatePtr;//current的下一个节点（终点方向）(常规意义上的上一个节点)   
//     //在这个函数里没啥用，不过还是维护了

//     GridNodePtr lastPtr;//current的上一个节点（起点方向）（常规意义上的下一个节点）

//     GridNodePtr lastTurningPtr=terminatePtr;//上一个关键点

//     gridPath.push_back(lastTurningPtr);//终点肯定是关键点

//     int line_point_count=0;
//     int temp_count=0;
//     while(currentPtr->cameFrom!=NULL)
//     {
//         GridNodePtr maxPtr=currentPtr->cameFrom;//初始化最远无碰节点
//         while(currentPtr->cameFrom!=NULL)
//         {
//             temp_count++;
//             lastPtr=currentPtr->cameFrom;   

//             int  x1 = lastTurningPtr->index(0);
//             int  y1 = lastTurningPtr->index(1);
//             int  z1 = lastTurningPtr->index(2);
//             int  x2 = lastPtr->index(0);
//             int  y2 = lastPtr->index(1);
//             int  z2 = lastPtr->index(2);

//             int collision_flag=0;//碰撞标志位

//             double divide_piece_num=20;//碰撞检测划分份数
//             for (double k=0;k<1;k+=1.0/divide_piece_num)
//             {
//                 //得到等分点坐标
//                 int x_check=int(x1*resolution_ratio+(double)k*(x2-x1)*resolution_ratio);
//                 int y_check=int(y1*resolution_ratio+(double)k*(y2-y1)*resolution_ratio);
//                 int z_check=int(z1*resolution_ratio+(double)k*(z2-z1)*resolution_ratio);
//                 // ROS_INFO("check_x=%d   y=%d   z=%d",x_check,y_check,z_check);
//                 if(if_collision(x_check,y_check,z_check))
//                     collision_flag=1;
//             }
//             if(collision_flag==0)
//             {
//                 maxPtr=lastPtr;//若无碰，更新最大无碰节点
//                 line_point_count=temp_count;
//             }

//             // nextPtr=currentPtr;//更新next
//             currentPtr=lastPtr;//更新current

//         }

//         double default_resolution=0.2;
//         int point_gap_max=3*(int)(default_resolution/resolution);//关键点间最大间隔数

//         if(line_point_count>point_gap_max)//如果直线太长，等分成若干份
//             {
//                 int divide_num=(int)line_point_count/point_gap_max+1;
//                 int gap=(int)line_point_count/divide_num;
//                 int temp_count=0;
//                 // GridNodePtr tempPtr=lastTurningPtr;
                


//                 int  x_last = lastTurningPtr->index(0);
//                 int  y_last = lastTurningPtr->index(1);
//                 int  z_last = lastTurningPtr->index(2);
//                 int  x_current = maxPtr->index(0);
//                 int  y_current = maxPtr->index(1);
//                 int  z_current = maxPtr->index(2);

//                 for (int i=1;i<divide_num;i++)//i从1开始避免重点
//                 {
//                     Vector3i temp_idx;
//                     temp_idx(0)=(int)(x_last+(double)(x_current-x_last)*(double)i/divide_num);
//                     temp_idx(1)=(int)(y_last+(double)(y_current-y_last)*(double)i/divide_num);
//                     temp_idx(2)=(int)(z_last+(double)(z_current-z_last)*(double)i/divide_num);

//                      GridNodePtr pushPtr = new GridNode(temp_idx, gridIndex2coord(temp_idx));
//                      gridPath.push_back(pushPtr);
//                 }

//             }

    

//         lastTurningPtr=maxPtr;//更新最新的关键点
//         temp_count=0;
//         line_point_count=0;
//         gridPath.push_back(lastTurningPtr);//储存关键点
//         currentPtr=maxPtr;//更新current

//         // break;
//     }
//     // gridPath.push_back(currentPtr);
    
//     nav_msgs::Path waypoints;
//     geometry_msgs::PoseStamped pt;

//     for (auto ptr: gridPath)
//     {
//         path.push_back(ptr->coord);//维护path
//         ROS_INFO("coord_x=%f   y=%f   z=%f",ptr->coord(0),ptr->coord(1),ptr->coord(2));
//     }
        
//     reverse(path.begin(),path.end());//这步在可视化上没有区别
//     return path;

// }


bool AstarPathFinder::if_collision(int x,int y,int z)
{
    return data_high_resolution[x * GLYZ_SIZE*resolution_ratio*resolution_ratio + y * GLZ_SIZE *resolution_ratio+ z ];
}


vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path, double path_resolution)
{
    vector<Vector3d> subPath;
    if (path.size() == 1 || path.size() == 2)
    {
        subPath = path;
    }
    else if (path.size() >= 3)
    {
        vector<Vector3d>::const_iterator maxIt;
        double maxDist = -INFINITY, tempDist;
        Vector3d vec((path.back() - path.front()).normalized());

        for (auto it = path.begin() + 1; it != (path.end() - 1); it++)
        {
            tempDist = (*it - path.front() - vec.dot(*it - path.front()) * vec).norm();
            if (maxDist < tempDist)
            {
                maxDist = tempDist;
                maxIt = it;
            }
        }

        if (maxDist > path_resolution)
        {
            subPath.insert(subPath.end(), path.begin(), maxIt + 1);
            subPath = pathSimplify(subPath, path_resolution);
            vector<Vector3d> tempPath(maxIt, path.end());
            tempPath = pathSimplify(tempPath, path_resolution);
            subPath.insert(subPath.end(), tempPath.begin() + 1, tempPath.end());
        }
        else
        {
            subPath.push_back(path.front());
            subPath.push_back(path.back());
        }
    }
    return subPath;
}


nav_msgs::Path AstarPathFinder::vector3d_to_waypoints(vector<Vector3d> path)
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



//输入A星得到的路径点，输出简化后的关键点,通过找拐点的形式
vector<Vector3d> AstarPathFinder::getSimplifiedPoints_by_lines()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr currentPtr=terminatePtr;//从终点开始往前找

    // GridNodePtr nextPtr=terminatePtr;//current的下一个节点（终点方向）(常规意义上的上一个节点)   
    //在这个函数里没啥用，不过还是维护了

    GridNodePtr lastPtr;//current的上一个节点（起点方向）（常规意义上的下一个节点）

    GridNodePtr lastTurningPtr=terminatePtr;//上一个关键点

    gridPath.push_back(lastTurningPtr);//终点肯定是关键点/

    double last_K_xy=0;
    double current_K_xy=0;
    double last_K_yz=0;
    double current_K_yz=0;

    double my_inf=9999;
    bool K_init_flag=1;
    int line_point_count=0;

    double default_resolution=0.2;
    int point_gap_max=3*(int)(default_resolution/resolution);//关键点间最大间隔数
    // int point_gap_max=4;
    // ROS_INFO("point_gap_max=%d   resolution=%f",point_gap_max,resolution);


    while(currentPtr->cameFrom!=NULL)
    {
        lastPtr=currentPtr->cameFrom;   

        int  x1 = lastTurningPtr->index(0);
        int  y1 = lastTurningPtr->index(1);
        int  z1 = lastTurningPtr->index(2);
        int  x2 = lastPtr->index(0);
        int  y2 = lastPtr->index(1);
        int  z2 = lastPtr->index(2);

        if(x2==x1)
        {
            current_K_xy=my_inf;
        }
        else
        {
            current_K_xy=(double)(y2-y1)/(x2-x1);
        }

        if(z2==z1)
        {
            current_K_yz=my_inf;
        }
        else
        {
            current_K_yz=(double)(y2-y1)/(z2-z1);
        }

        if(K_init_flag)//斜率的初始化
        {
            last_K_xy=current_K_xy;
            last_K_yz=current_K_yz;
            K_init_flag=0;
        }        

        // ROS_INFO("x1=%d  x2=%d  y1=%d  y2=%d  current_K=%f   last_K=%f",x1,x2,y1,y2,current_K,last_K);

        if(current_K_xy!=last_K_xy||current_K_yz!=last_K_yz)//斜率不同时，说明当前点的上一个点（即还没更新的current）是拐点
        {
            if(line_point_count>point_gap_max)//如果直线太长，等分成若干份
            {
                int divide_num=(int)line_point_count/point_gap_max+1;
                int gap=(int)line_point_count/divide_num;
                int temp_count=0;
                GridNodePtr tempPtr=lastTurningPtr;
                while(tempPtr!=currentPtr)
                {
                    temp_count++;
                    if(temp_count%gap==0)
                        gridPath.push_back(tempPtr);
                    tempPtr=tempPtr->cameFrom;
                }
            }
            


            // ROS_INFO("line_point_count=%d",line_point_count);
            line_point_count=0;
            lastTurningPtr=currentPtr;
            gridPath.push_back(lastTurningPtr);
            K_init_flag=1;
            continue;
        }
        else
        {
            line_point_count++;
            // nextPtr=currentPtr;//更新next
            currentPtr=lastPtr;//更新current
        }
    }
    gridPath.push_back(currentPtr);
    
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    for (auto ptr: gridPath)
    {
        path.push_back(ptr->coord);//维护path
        // ROS_INFO("coord_x=%f   y=%f   z=%f",ptr->coord(0),ptr->coord(1),ptr->coord(2));
    }
        
    reverse(path.begin(),path.end());//这步在可视化上没有区别
    return path;

}

int my_inf=9999;
bool tra_collision_flag=1;
int first_key_point=my_inf;
int second_key_point=my_inf;



//迭代增加关键点
vector<Vector3d> AstarPathFinder::recursive_get_simplified_points(vector<Vector3d>raw_path,visualization_msgs::Marker traj,int &flag)
{
    static int count=0;
    count++;
    if(count==1)
    {
        count=0;
        flag=0;
    }
    // flag=0;
    auto points=traj.points;
    ROS_INFO("traj size= %d ",points.size());

    int coll_order=0;
    int collision_flag=0;
    for (int i=0;i<points.size();i++)
    {
        Vector3d temp_coord(points[i].x,points[i].y,points[i].z);
        // temp_coord
        // ROS_INFO("traj X=%f  Y=%f  Z=%f  ",points[i].x,points[i].y,points[i].z);
        // cout<<typeid(points[i]).name()<<endl;
        auto temp_idx=coord2gridIndex(temp_coord);
        if(isOccupied(temp_idx[0],temp_idx[1],temp_idx[2]))
        {
            collision_flag=1;

            double max_cos_theta=-100;

            for (i=0;i<raw_path.size()-1;i++)
            {
                double dx1=temp_coord(0)-raw_path[i](0);
                double dy1=temp_coord(1)-raw_path[i](1);
                double dz1=temp_coord(2)-raw_path[i](2);
                double d1=sqrt(dx1*dx1+dy1*dy1+dz1*dz1);

                double dx2=temp_coord(0)-raw_path[i+1](0);
                double dy2=temp_coord(1)-raw_path[i+1](1);
                double dz2=temp_coord(2)-raw_path[i+1](2);
                double d2=sqrt(dx2*dx2+dy2*dy2+dz2*dz2);

                double dx3=raw_path[i+1](0)-raw_path[i](0);
                double dy3=raw_path[i+1](1)-raw_path[i](1);
                double dz3=raw_path[i+1](2)-raw_path[i](2);
                double d3=sqrt(dx3*dx3+dy3*dy3+dz3*dz3);

                double cos_theta=(d3*d3-d1*d1-d2*d2)/(2*d1*d2);

                ROS_INFO("cos_theta=%f   ",cos_theta);

                if(cos_theta>max_cos_theta)
                {
                    max_cos_theta=cos_theta;
                    coll_order=i;
                }

            }
            break;
        }
    }

    if(collision_flag)
    {
        ROS_WARN("collision!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        double temp_x=(raw_path[coll_order+1](0)+raw_path[coll_order](0))/2;
        double temp_y=(raw_path[coll_order+1](1)+raw_path[coll_order](1))/2;
        double temp_z=(raw_path[coll_order+1](2)+raw_path[coll_order](2))/2;
        Vector3d temp_p(temp_x,temp_y,temp_z);
        auto loc=raw_path.begin()+coll_order;
        raw_path.insert(loc,temp_p);
        ROS_WARN("insertion suc!");
    }

    // ros::NodeHandle nh("~");
    // ros::Subscriber trajectory_sub;
    // trajectory_sub  = nh.subscribe( "/trajectory_generator_node/vis_trajectory",       1, TrajectoryCallBack );

    // ros::Rate rate(100);
    // bool status = ros::ok();
    // while(status) 
    // {
    //     ros::spinOnce();      
    //     status = ros::ok();
    //     rate.sleep();
    // }

    // path_pub(raw_path);
    return raw_path;

}