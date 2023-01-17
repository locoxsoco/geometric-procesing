#include <iostream>
#include "MongePatch.h"


// Given a point P, its normal, and its closest neighbors (including itself) 
// compute a quadratic Monge patch that approximates the neighborhood of P.
// The resulting patch will be used to compute the principal curvatures of the 
// surface at point P.

void MongePatch::init(const glm::vec3 &P, const glm::vec3 &normal, const vector<glm::vec3> &closest)
{
}

// Return the values of the two principal curvatures for this patch

void MongePatch::principalCurvatures(float &kmin, float &kmax) const
{
	kmin = 0.f;
	kmax = 0.f;
}


