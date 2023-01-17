#include "utils.h"

void pca(const vector<glm::vec3>& points, vector<size_t>& neighbor_ids, vector<float>& dists_squared, Eigen::Matrix3f& L, Eigen::Vector3f& D) {
	// 1. Compute Centroid
	glm::vec3 p_centroid = getCentroid(points, neighbor_ids);

	// 2. Translate all points setting p_centroid as origin
	vector<glm::vec3> neighbors_corigin = getNeighborsCentered(points, p_centroid, neighbor_ids);

	// 3. Build Covariance Matrix
	Eigen::Matrix3f C;
	C = Eigen::Matrix3f::Zero();
	for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
		C += Eigen::Vector3f(
			neighbors_corigin[j][0],
			neighbors_corigin[j][1],
			neighbors_corigin[j][2]
		) * Eigen::Vector3f(
			neighbors_corigin[j][0],
			neighbors_corigin[j][1],
			neighbors_corigin[j][2]
		).transpose();
	}

	// 4. Compute Spectral Decomposition
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(C);
	L = eigensolver.eigenvectors();
	D = eigensolver.eigenvalues();

	// 5. Sort eigenvalues in decreasing order
	// L and D returns values already sorted in decreasing order

	// 6. Get PCA eigenvectors to compute normal, etc
	return;
}

glm::vec3 getCentroid(const vector<glm::vec3>& points) {
	glm::vec3 p_centroid(0.0, 0.0, 0.0);
	float total_dists_squared = 0.f;
	for (unsigned int j = 0; j < points.size(); j++) {
		p_centroid += points[j];
	}
	return p_centroid /= points.size();
}

glm::vec3 getCentroid(const vector<glm::vec3>& points, vector<size_t>& neighbor_ids) {
	glm::vec3 p_centroid(0.0, 0.0, 0.0);
	float total_dists_squared = 0.f;
	for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
		p_centroid += points[neighbor_ids[j]];
	}
	return p_centroid /= neighbor_ids.size();
}

glm::vec3 getCentroid(const vector<glm::vec3>& points1, const vector<glm::vec3>& points2, vector<int>& correspondence) {
	glm::vec3 p_centroid(0.0, 0.0, 0.0);
	float total_dists_squared = 0.f;
	for (unsigned int j = 0; j < points2.size(); j++) {
		p_centroid += points1[correspondence[j]];
	}
	return p_centroid /= points2.size();
}

vector<glm::vec3> getPointsCentered(const vector<glm::vec3>& points, glm::vec3 centroid) {
	vector<glm::vec3> points_centered(points.size());
	for (unsigned int j = 0; j < points.size(); j++) {
		points_centered[j] = points[j] - centroid;
	}
	return points_centered;
}

vector<glm::vec3> getNeighborsCentered(const vector<glm::vec3>& points, glm::vec3 centroid, vector<size_t>& neighbor_ids) {
	vector<glm::vec3> neighbors_centered(neighbor_ids.size());
	for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
		neighbors_centered[j] = points[neighbor_ids[j]] - centroid;
	}
	return neighbors_centered;
}

vector<glm::vec3> getPointsCentered(const vector<glm::vec3>& points1, glm::vec3 centroid, const vector<glm::vec3>& points2, vector<int>& correspondence) {
	vector<glm::vec3> points_centered(points2.size());
	for (unsigned int j = 0; j < points2.size(); j++) {
		points_centered[j] = points1[correspondence[j]] - centroid;
	}
	return points_centered;
}