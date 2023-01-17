#ifndef _UTILS_INCLUDE
#define _UTILS_INCLUDE

#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Dense>

using namespace std;

glm::vec3 getCentroid(const vector<glm::vec3>& points);
glm::vec3 getCentroid(const vector<glm::vec3>& points1, const vector<glm::vec3>& points2, vector<int>& correspondence);
glm::vec3 getCentroid(const vector<glm::vec3>& points, vector<size_t>& neighbor_ids);
glm::vec3 getCentroid(const vector<glm::vec3>& points, vector< pair<size_t, float> >& neighbor_ids);
vector<glm::vec3> getPointsCentered(const vector<glm::vec3>& points, glm::vec3 centroid);
vector<glm::vec3> getPointsCentered(const vector<glm::vec3>& points1, glm::vec3 centroid, const vector<glm::vec3>& points2, vector<int>& correspondence);
vector<glm::vec3> getNeighborsCentered(const vector<glm::vec3>& points, glm::vec3 centroid, vector<size_t>& neighbor_ids);
vector<glm::vec3> getNeighborsCentered(const vector<glm::vec3>& points, glm::vec3 centroid, vector< pair<size_t, float> >& neighbor_ids);
void pca(const vector<glm::vec3>& points, vector<size_t>& neighbor_ids, vector<float>& dists_squared, Eigen::Matrix3f& L, Eigen::Vector3f& D);
void pca(const vector<glm::vec3>& points, vector< pair<size_t, float> >& neighbor_ids, Eigen::Matrix3f& L, Eigen::Vector3f& D);

#endif // _UTILS_INCLUDE