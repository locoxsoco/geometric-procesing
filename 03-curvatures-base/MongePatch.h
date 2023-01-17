#ifndef _MONGE_PATCH_INCLUDE
#define _MONGE_PATCH_INCLUDE


#include <vector>
#include "glm/glm.hpp"
#include <Eigen/Core>
#include <Eigen/SVD>


using namespace std;


class MongePatch
{

public:
	void init(const glm::vec3 &P, const glm::vec3 &normal, const vector<glm::vec3> &closest);
	
	void principalCurvatures(float &kmin, float &kmax) const;

	vector<float> vs;
	glm::vec3 vertex;
	
};


#endif // _MONGE_PATCH_INCLUDE


