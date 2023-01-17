#include "SimpleDistance.h"


/* Initialize everything to be able to compute the implicit distance of [Hoppe92] 
   at arbitrary points that are close enough to the point cloud.
 */

void SimpleDistance::init(const PointCloud *pointCloud, float samplingRadius)
{
	cloud = pointCloud;
	nn.setPoints(&(pointCloud->getPoints()));
	nn_radius = samplingRadius;
	return;
}


/* This operator returns a boolean that if true signals that the value parameter
   has been modified to contain the value of the implicit function of [Hoppe92]
   at point P.
 */

bool SimpleDistance::operator()(const glm::vec3 &P, float &value) const
{
	vector< pair<size_t, float> > neighbor_ids;
	nn.getNeighborsInRadius(P, nn_radius, neighbor_ids);
	// i <- Index of pi, closest point to p
	int i = neighbor_ids[0].first;
	//{ Compute z as the projection of p onto the tangent plane at pi }
	glm::vec3 pi = cloud->getPoints()[i];
	glm::vec3 ni = cloud->getNormals()[i];
	glm::vec3 z = pi - (glm::dot((P - pi), ni))*ni;
	if (glm::length(z - pi) <= rho + delta) {
		value = glm::dot((P - pi), ni);
		return true;
	}
	return false;
}






