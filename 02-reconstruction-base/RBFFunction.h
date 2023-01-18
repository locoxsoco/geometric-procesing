#ifndef _RBF_FUNCTION_INCLUDE
#define _RBF_FUNCTION_INCLUDE


#include "ImplicitFunction.h"
#include "PointCloud.h"
#include "NearestNeighbors.h"


class RBFFunction : public ImplicitFunction
{

public:
	void init(const PointCloud *pointCloud, float standardDeviation, float supportRadius);

	bool operator()(const glm::vec3 &P, float &value) const;
	
private:
	const PointCloud* cloud;
	vector<double> vc;
	NearestNeighbors nn;
	float std_dv,nn_radius;
	float rho = 1.0, delta = 1.0, lambda_regularization=0.3, d = 0.0001;

	double gaussianRBF(double r) const;
};


#endif // _RBF_FUNCTION_INCLUDE


