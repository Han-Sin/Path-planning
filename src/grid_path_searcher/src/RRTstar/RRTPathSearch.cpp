#include "RRTPathSearch.h"
using namespace std;
using namespace Eigen;

bool RRTPathSearch::safe_check(Vector3i pos1,Vector3i pos2){
    //返回1安全，返回0不安全
    int  x1 = pos1(0);
    int  y1 = pos1(1);
    int  z1 = pos1(2);
    int  x2 = pos2(0);
    int  y2 = pos2(1);
    int  z2 = pos2(2);
    int collision_flag=0;//碰撞标志位
    int divide_piece_num = 5;
    for (double k=0;k<1;k+=1.0/5){
        //得到等分点坐标    原始地图
        int x_check=int(x1+(double)k*(x2-x1));
        int y_check=int(y1+(double)k*(y2-y1));
        int z_check=int(z1+(double)k*(z2-z1));
        if(isOccupied(x_check,y_check,z_check)){
            collision_flag=1;
            break;
        }        
    }
    return !collision_flag;
}




void RRTPathSearch::RRTSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt){
    //(rand() % (b-a))+ a    [a,b) ,本RRT全部采用栅格地图
    ros::Time time_1 = ros::Time::now();
    double step = 0.5;//步长
    double threshold = 1;
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    vector<GridNodePtr> visited_grid;

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->id = -1;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->gScore = 0;
    visited_grid.push_back(GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]);
    int sample_x,sample_y,sample_z;
    while(1){
        double min_distance =  INF;
        double a;
        while(1){ 
            a = (rand()%10000)/10000.0;
            if(a>0.3){
                sample_x = end_idx(0);
                sample_y = end_idx(1);
                sample_z = end_idx(2);
            }//目标导向
            else{
                sample_x = rand()%GLX_SIZE;
                sample_y = rand()%GLY_SIZE;
                sample_z = rand()%GLZ_SIZE;
            }
            if((GridNodeMap[sample_x][sample_y][sample_z]->id!=-1)&&(!isOccupied(sample_x,sample_y,sample_z)))
                break;//保证采样的有效性
        }
        Vector3d Sample_coord;
        Vector3i Sample_idx;
        Sample_idx(0) = sample_x;
        Sample_idx(1) = sample_y;
        Sample_idx(2) = sample_z;
        Sample_coord = gridIndex2coord(Sample_idx);
        Vector3i tempidx;
        GridNodePtr ptr;
        //开始拓展树,遍历所有结点
        for (auto tree_node: visited_grid)
        {
                        //在树上的结点
            double distance;
            distance =  (tree_node->coord-Sample_coord).norm();
            if(distance<min_distance){
                //find the min distance node
                min_distance = distance;
                tempidx(0) = tree_node->index(0);
                tempidx(1) = tree_node->index(1);
                tempidx(2) = tree_node->index(2);
                ptr = tree_node;
            }
        }

        if(min_distance<=step){
            if(!safe_check(tempidx,Sample_idx)) continue;
            visited_grid.push_back(GridNodeMap[Sample_idx(0)][Sample_idx(1)][Sample_idx(2)]);
            GridNodeMap[Sample_idx(0)][Sample_idx(1)][Sample_idx(2)]->id = -1;
            GridNodeMap[Sample_idx(0)][Sample_idx(1)][Sample_idx(2)]->cameFrom = ptr;
            GridNodeMap[Sample_idx(0)][Sample_idx(1)][Sample_idx(2)]->gScore = ptr->gScore+min_distance;
            terminatePtr = GridNodeMap[Sample_idx(0)][Sample_idx(1)][Sample_idx(2)];
            if(Sample_idx(0)==end_idx(0)&&Sample_idx(1)==end_idx(1)&&Sample_idx(2)==end_idx(2))//到达目标 
                {
                    // ros::Time time_2 = ros::Time::now();
                    // ROS_WARN("Time consume in RRT* path finding is %f ms", (time_2 - time_1).toSec()*1000 );
                    break;
            }
        }
        else{
            //直线拓展
            Vector3d tempcoord = gridIndex2coord(tempidx);//选取的拓展节点的坐标
            Vector3d expand_coord;
            Vector3i expand_idx;
            expand_coord = (Sample_coord-tempcoord)*step/min_distance+tempcoord;
            expand_idx = coord2gridIndex(expand_coord);
            if(!safe_check(tempidx,expand_idx)) continue;
            visited_grid.push_back(GridNodeMap[expand_idx(0)][expand_idx(1)][expand_idx(2)]);
            GridNodeMap[expand_idx(0)][expand_idx(1)][expand_idx(2)]->id=-1;
            GridNodeMap[expand_idx(0)][expand_idx(1)][expand_idx(2)]->cameFrom = ptr;
            GridNodeMap[expand_idx(0)][expand_idx(1)][expand_idx(2)]->gScore = ptr->gScore+step;
            terminatePtr = GridNodeMap[expand_idx(0)][expand_idx(1)][expand_idx(2)];
            if(expand_idx(0)==end_idx(0)&&expand_idx(1)==end_idx(1)&&expand_idx(2)==end_idx(2)) break;
        }
        //for (auto ptr: gridPath)
        //path.push_back(ptr->coord)
        //RRT*部分，进行分支改造
        for(GridNodePtr ptr:visited_grid){
            if(ptr!=terminatePtr){
                //不是自己
                double d = (ptr->coord-terminatePtr->coord).norm();
                if(d<threshold){
                    if(ptr->gScore+d<terminatePtr->gScore){
                        terminatePtr->gScore = ptr->gScore+d;
                        terminatePtr->cameFrom = ptr;
                    }
                }    
            }
        }
    }
    //terminatePtr = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("[RRT*]{sucess}  Time in RRT*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, terminatePtr->gScore);
    //ROS_WARN("visited grid size=%d",visited_grid.size());
}