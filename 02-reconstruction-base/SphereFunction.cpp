#include "SphereFunction.h"


void SphereFunction::init(const glm::vec3 &center, float radius)
{
	C = center;
	R = radius;
}

bool SphereFunction::operator()(const glm::vec3 &P, float &value) const
{
	glm::vec3 ni = glm::normalize(P-C);
	glm::vec3 pi = C + ni * R;
	glm::vec3 z = pi - (glm::dot((P - pi), ni)) * ni;
	if (glm::length(z - pi) <= rho + delta) {
		//value = glm::dot((P - pi), ni);
		value = glm::distance(P, C) - R;
		return true;
	}
	//value = glm::distance(P, C) - R;
	
	return false;
}




