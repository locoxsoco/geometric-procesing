#ifndef _ITERATIVE_CLOSEST_POINT_INCLUDE
#define _ITERATIVE_CLOSEST_POINT_INCLUDE


#include "PointCloud.h"
#include "NearestNeighbors.h"
#include "utils.h"


class IterativeClosestPoint
{

public:
	void setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2);
	
	void markBorderPoints(PointCloud &cloud);
	vector<int> *computeCorrespondence();
	glm::mat4 computeICPStep();
	
	vector<int> *computeFullICP(unsigned int maxSteps = 100);
	
private:
	PointCloud *cloud1, *cloud2;
	vector<int> border_points;
	vector<int> correspondence;

};


#endif // _ITERATIVE_CLOSEST_POINT_INCLUDE


