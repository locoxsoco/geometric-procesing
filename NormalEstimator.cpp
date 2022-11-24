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
		unsigned int kNeighbors = 8;
		vector<size_t> neighbor_ids;
		vector<float> dists_squared;
		nn.getKNearestNeighbors(points[i], kNeighbors, neighbor_ids, dists_squared);

		// PCA Method
		glm::vec3 normal = pca(points, neighbor_ids, dists_squared);
		if (normals[i].z < 0)
			normals[i] = -normal;
		else normals[i] = normal;
	}
}


