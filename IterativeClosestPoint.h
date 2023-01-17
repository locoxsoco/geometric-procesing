#ifndef _ITERATIVE_CLOSEST_POINT_INCLUDE
#define _ITERATIVE_CLOSEST_POINT_INCLUDE

#include <limits>
#include "PointCloud.h"
#include "NearestNeighbors.h"
#include "utils.h"

class IterativeClosestPoint
{

public:
	void setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2);
	
	void markBorderPoints();
	vector<int> *computeCorrespondence();
	glm::mat4 computeICPStep();
	
	bool hasConverged(glm::mat4 matrix);

	vector<int> *computeFullICP(unsigned int maxSteps = 10);
	
private:
	PointCloud *cloud1, *cloud2;
	vector<int> border_points;
	vector<int> correspondence;
	float convergence_threshold = 0.1;
	float max_delta_alpha_threshold = 2.f;
	unsigned int kNeighbors = 13;
	NearestNeighbors knn1;

};


#endif // _ITERATIVE_CLOSEST_POINT_INCLUDE


