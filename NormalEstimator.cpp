#include "NormalEstimator.h"
#include "NearestNeighbors.h"


// This method has to compute a normal per point in the 'points' vector and put it in the 
// 'normals' vector. The 'normals' vector already has the same size as the 'points' vector. 
// There is no need to push_back new elements, only to overwrite them ('normals[i] = ...;')
// In order to compute the normals you should use PCA. The provided 'NearestNeighbors' class
// wraps the nanoflann library that computes K-nearest neighbors effciently. 

glm::vec3 pca(const vector<glm::vec3>& points, vector<size_t>& neighbor_ids) {
	// 1. Compute Centroid
	glm::vec3 p_centroid(0.0, 0.0, 0.0);
	for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
		p_centroid += points[neighbor_ids[j]];
	}
	p_centroid /= neighbor_ids.size();

	// 2. Translate all points setting p_centroid as origin
	vector<glm::vec3> neighbors_corigin(neighbor_ids.size());
	for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
		neighbors_corigin[j] = points[neighbor_ids[j]] - p_centroid;
	}

	// 3. Build Covariance Matrix
	Eigen::Matrix3f C;
	C = Eigen::Matrix3f::Zero();
	for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
		C += Eigen::Vector3f(
			points[neighbor_ids[j]][0],
			points[neighbor_ids[j]][1],
			points[neighbor_ids[j]][2]
		) * Eigen::Vector3f(
			points[neighbor_ids[j]][0],
			points[neighbor_ids[j]][1],
			points[neighbor_ids[j]][2]
		).transpose();
	}

	// 4. Compute Spectral Decomposition
	// Since matrix is small and definite semi-positive, I'll use Cholesky
	Eigen::LDLT<Eigen::Matrix3f> ldltOfA(C);
	Eigen::Matrix3f L = ldltOfA.matrixL();
	Eigen::Vector3f D = ldltOfA.vectorD();

	// 5. Sort eigenvalues in decreasing order
	unsigned int d_ids[3] = { 0,1,2 };

	// Insert arr[1]
	if (D[d_ids[1]] > D[d_ids[0]])
		swap(d_ids[0], d_ids[1]);
	// Insert arr[2]
	if (D[d_ids[2]] > D[d_ids[1]])
	{
		swap(d_ids[1], d_ids[2]);
		if (D[d_ids[1]] > D[d_ids[0]])
			swap(d_ids[1], d_ids[0]);
	}

	// 6. Get PCA eigenvectors to compute normal;
	vector<glm::vec3> eigens;
	eigens.push_back(glm::vec3(L(0, 0), L(1, 0), L(2, 0)));
	eigens.push_back(glm::vec3(L(1, 0), L(1, 1), L(2, 1)));
	eigens.push_back(glm::vec3(L(2, 0), L(2, 1), L(2, 2)));
	return glm::cross(eigens[d_ids[0]], eigens[d_ids[1]]);
}

void NormalEstimator::computePointCloudNormals(const vector<glm::vec3> &points, vector<glm::vec3> &normals)
{
	// TODO

	// Create de KD-tree nearest neighbors structure
	NearestNeighbors nn;
	nn.setPoints(&points);

	vector<size_t> points_ids;
	for (unsigned int i = 0; i < points.size(); i++) {
		points_ids.push_back(i);
	}
	glm::vec3 gnormal = pca(points, points_ids);

	for (unsigned int i = 0; i < points.size(); i++) {
		// Get the KNN for the point
		unsigned int kNeighbors = 20;
		vector<size_t> neighbor_ids;
		vector<float> dists_squared;
		nn.getKNearestNeighbors(points[i], kNeighbors, neighbor_ids, dists_squared);

		// PCA Method
		glm::vec3 normal = pca(points, neighbor_ids);
		normals[i] = normal;
		/*if (glm::dot(normals[i], gnormal) < 0)
			normals[i] *= -1;*/
		if (normals[i].z < 0)
			normals[i] *= -1;
	}
}


