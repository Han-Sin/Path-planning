#ifndef _GRIDNODE_H_
#define _GRIDNODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index;
    Eigen::Matrix<double,6,1> state;
    Eigen::Vector3d input;
    double duration;
    double time;
    int time_idx;
    char node_state;
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();
    node_state = NOT_EXPAND;
		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }
    GridNode(){
      cameFrom = NULL;
      node_state = NOT_EXPAND;
    };
    ~GridNode(){};
};


#endif
