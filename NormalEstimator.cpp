#include "NormalEstimator.h"
#include "NearestNeighbors.h"


// This method has to compute a normal per point in the 'points' vector and put it in the 
// 'normals' vector. The 'normals' vector already has the same size as the 'points' vector. 
// There is no need to push_back new elements, only to overwrite them ('normals[i] = ...;')
// In order to compute the normals you should use PCA. The provided 'NearestNeighbors' class
// wraps the nanoflann library that computes K-nearest neighbors effciently. 

void NormalEstimator::computePointCloudNormals(const vector<glm::vec3> &points, vector<glm::vec3> &normals)
{
	// TODO

	// Create de KD-tree nearest neighbors structure
	NearestNeighbors nn;
	nn.setPoints(&points);

	for (unsigned int i = 0; i < points.size(); i++) {
		// Get the KNN for the point
		unsigned int kNeighbors = 13;
		vector<size_t> neighbor_ids;
		vector<float> dists_squared;
		nn.getKNearestNeighbors(points[i], kNeighbors, neighbor_ids, dists_squared);

		// PCA Method
		Eigen::Matrix3f L;
		Eigen::Vector3f D;
		pca(points, neighbor_ids, dists_squared, L ,D);
		glm::vec3 normal = glm::vec3(L(0, 0), L(1, 0), L(2, 0));
		if (normals[i].z < 0)
			normals[i] = -normal;
		else normals[i] = normal;
	}
}


