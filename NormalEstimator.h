#ifndef _NORMAL_ESTIMATOR_INCLUDE
#define _NORMAL_ESTIMATOR_INCLUDE


#include <vector>
#include <glm/glm.hpp>
#include <iostream>
#include "utils.h"


using namespace std;


class NormalEstimator
{

public:
	void computePointCloudNormals(const vector<glm::vec3> &points, vector<glm::vec3> &normals);
	unsigned int kNeighbors = 13;

};


#endif // _NORMAL_ESTIMATOR_INCLUDE


