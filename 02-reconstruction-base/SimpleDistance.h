#ifndef _SIMPLE_DISTANCE_INCLUDE
#define _SIMPLE_DISTANCE_INCLUDE


#include <glm/glm.hpp>
#include "ImplicitFunction.h"
#include "PointCloud.h"
#include "NearestNeighbors.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include "utils.h"


class SimpleDistance : public ImplicitFunction
{

public:
	void init(const PointCloud *pointCloud, float samplingRadius);

	bool operator()(const glm::vec3 &P, float &value) const;
	
private:
	const PointCloud* cloud;
	NearestNeighbors nn;
	float nn_radius;
	float rho = 1, delta= 1;

};


#endif // _SIMPLE_DISTANCE_INCLUDE



