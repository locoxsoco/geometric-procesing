#ifndef _UTILS_INCLUDE
#define _UTILS_INCLUDE

#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Dense>

using namespace std;

glm::vec3 getCentroid(const vector<glm::vec3>& points);
glm::vec3 getCentroid(const vector<glm::vec3>& points, vector<size_t>& neighbor_ids);
vector<glm::vec3> getPointsCentered(const vector<glm::vec3>& points, glm::vec3 centroid);
vector<glm::vec3> getNeighborsCentered(const vector<glm::vec3>& points, glm::vec3 centroid, vector<size_t>& neighbor_ids);
glm::vec3 pca(const vector<glm::vec3>& points, vector<size_t>& neighbor_ids, vector<float>& dists_squared);

#endif // _UTILS_INCLUDE