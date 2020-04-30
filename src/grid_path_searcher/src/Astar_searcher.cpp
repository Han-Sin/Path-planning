#include "Astar_searcher.h"
#include <math.h>
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

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);

    int expand_size=1;

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


//    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;//index(grid)
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

    while(terminatePtr!=NULL){
        gridPath.push_back(terminatePtr);
        terminatePtr = terminatePtr->cameFrom;
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
                neighborPtr->gScore=currentPtr->gScore+1;
//                neighborPtr->gScore=currentPtr->gScore+getHeu(neighborPtr,currentPtr);
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
                if(neighborPtr->gScore>currentPtr->gScore+1){
                    //change
                    neighborPtr->gScore = currentPtr->gScore+1;
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
                if(neighborPtr->gScore>currentPtr->gScore+1){
                    neighborPtr->gScore = currentPtr->gScore+1;
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