#ifndef _RRTPATHSEARCH_H_
#define _RRTPATHSEARCH_H_

#include "Astar_searcher.h"

class RRTPathSearch:public AstarPathFinder
{
private:
  
public:
	RRTPathSearch(){
		srand((unsigned)time(NULL)); 
	};
	bool safe_check(Eigen::Vector3i pos1,Eigen::Vector3i pos2);
	~RRTPathSearch(){};
	void RRTSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
};



#endif