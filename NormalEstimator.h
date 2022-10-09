#ifndef _NORMAL_ESTIMATOR_INCLUDE
#define _NORMAL_ESTIMATOR_INCLUDE


#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <iostream>


using namespace std;


class NormalEstimator
{

public:
	void computePointCloudNormals(const vector<glm::vec3> &points, vector<glm::vec3> &normals);

};


#endif // _NORMAL_ESTIMATOR_INCLUDE


