#include <graph_searcher.h>

using namespace std;
using namespace Eigen;

void RRTstarPreparatory::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void RRTstarPreparatory::setObs(const double coord_x, const double coord_y, const double coord_z)
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
        expand_size=0;
    
    // expand_size=expand_size+1;
    
    // expand_size=0;

    // if(idx_z==1)
        // ROS_INFO("obs x=%d y=%d  z=%d",idx_x,idx_y,idx_z);

    for (int i=-expand_size;i<=expand_size;i++)
        for (int j=-expand_size;j<=expand_size;j++)
            for (int k=-expand_size;k<=expand_size;k++)
            {
                // if(abs(i)+abs(j)+abs(k)==3*expand_size)//膨胀成十字形，而不是方块
                // {
                //     // ROS_INFO("i=%d  j=%d  k=%d   ",i,j,k);
                //     continue;
                // }
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
}

bool RRTstarPreparatory::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d RRTstarPreparatory::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i RRTstarPreparatory::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d RRTstarPreparatory::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}
