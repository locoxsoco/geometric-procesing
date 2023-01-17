#include <iostream>
#include <algorithm>
#include "IterativeClosestPoint.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <glm/gtc/matrix_transform.hpp>



void IterativeClosestPoint::setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2)
{
	cloud1 = pointCloud1;
	cloud2 = pointCloud2;
	knn1.setPoints(&(cloud1->getPoints()));
}

// This method should mark the border points in cloud 1. It also changes their color (for example to red).
// You will need to add an attribute to this class that stores this property for all points in cloud 1. 

void IterativeClosestPoint::markBorderPoints()
{
	// TODO
	const vector<glm::vec3>* points1 = &(cloud1->getPoints());

	for (unsigned int i = 0; i < cloud1->getPoints().size(); i++) {
		glm::vec3 point = cloud1->getPoints()[i];
		// 1. Compute its k-nearest neighbors
		unsigned int kNeighbors = 8;
		vector<size_t> neighbor_ids;
		vector<float> dists_squared;
		knn1.getKNearestNeighbors(point, kNeighbors, neighbor_ids, dists_squared);

		// 2. Use PCA to compute local reference frame for point p
		Eigen::Matrix3f L;
		Eigen::Vector3f D;
		pca(*points1, neighbor_ids, dists_squared, L, D);

		// 3. Transform the k-nearest neighbors to this frame
		vector<glm::vec3> neighbors_frame(neighbor_ids.size());
		for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
			glm::vec3 pj_pi = points1->at(neighbor_ids[j]) - point;
			neighbors_frame[j] = glm::vec3(
				glm::dot(pj_pi, glm::vec3(L(0, 0), L(0, 1), L(0, 2))),
				glm::dot(pj_pi, glm::vec3(L(1, 0), L(1, 1), L(1, 2))),
				glm::dot(pj_pi, glm::vec3(L(2, 0), L(2, 1), L(2, 2)))
			);
		}

		// 4. Project XY-plane
		for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
			glm::vec3 pj_pi = points1->at(neighbor_ids[j]) - point;
			neighbors_frame[j] = glm::vec3(
				pj_pi.x,
				pj_pi.y,
				0
			);
		}

		// 5. Compute angle of polar representation on plane
		vector<float> polar_representation_angles(neighbor_ids.size());
		for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
			polar_representation_angles[j] = atan2(neighbors_frame[j].y, neighbors_frame[j].x);
		}

		// 6. Sort the polar_representation
		std::sort(polar_representation_angles.begin(), polar_representation_angles.end());

		// 7. Look for largest difference between adjacent angles maxDeltaAlphai
		// If maxDeltaAlphai greater then threshold, pi is border point
		int is_border_point = 0;
		// Looking for an adjacent difference > threshold
		for (unsigned int j = 0; j < neighbor_ids.size()-1; j++) {
			if (polar_representation_angles[j+1] - polar_representation_angles[j] > max_delta_alpha_threshold) {
				is_border_point = 1;
				cloud1->getColors()[i] = glm::vec4(1.0,0.0,0.0,1.0);
			}
		}
		border_points.push_back(is_border_point);
	}
}


// This method should compute the closest point in cloud 1 for all non border points in cloud 2. 
// This correspondence will be useful to compute the ICP step matrix that will get cloud 2 closer to cloud 1.
// Store the correspondence in this class as the following method is going to need it.
// As it is evident in its signature this method also returns the correspondence. The application draws this if available.

vector<int> *IterativeClosestPoint::computeCorrespondence()
{
	// TODO
	correspondence.clear();
	unsigned int kNeighbors = 8;
	vector<size_t> neighbor_ids;
	vector<float> dists_squared;
	bool knn_found = false;
	int closest_point_id = -1;
	for (int i = 0; i < cloud2->getPoints().size(); i++) {
		// search with knn not border point
		knn1.getKNearestNeighbors(cloud2->getPoints()[i], kNeighbors, neighbor_ids, dists_squared);
		int j;
		for (j = 0; j < kNeighbors; j++) {
			if (!border_points[neighbor_ids[j]]) {
				knn_found = true;
				closest_point_id = neighbor_ids[j];
				break;
			}
		}
		// if not lucky apply brute force
		if (!knn_found) {
			float min_distance = std::numeric_limits<float>::max();
			for (j = 0; j < cloud1->getPoints().size(); j++) {
				float cur_distance = glm::length(cloud2->getPoints()[i] - cloud1->getPoints()[j]);
				if (!border_points[j] && min_distance > cur_distance) {
					min_distance = cur_distance;
					closest_point_id = j;
				}
			}
		}		
		correspondence.push_back(closest_point_id);
	}
	
	return &correspondence;
}


// This method should compute the rotation and translation of an ICP step from the correspondence
// information between clouds 1 and 2. Both should be encoded in the returned 4x4 matrix.
// To do this use the SVD algorithm in Eigen.

glm::mat4 IterativeClosestPoint::computeICPStep()
{
	// TODO
	const vector<glm::vec3>* points1 = &(cloud1->getPoints());
	const vector<glm::vec3>* points2 = &(cloud2->getPoints());

	// 1. Compute the centroids corrected versions of the point sets
	glm::vec3 q_centroid = getCentroid(*points2);
	vector<glm::vec3> q_corigin = getPointsCentered(*points2, q_centroid);

	glm::vec3 p_centroid = getCentroid(*points1, *points2, correspondence);
	vector<glm::vec3> p_corigin = getPointsCentered(*points1, p_centroid, *points2, correspondence);
	// 2. Compute the covariance matrix S of both points sets
	Eigen::MatrixX3f Q(p_corigin.size(),3), P(p_corigin.size(),3);
	for (unsigned int j = 0; j < q_corigin.size(); j++) {
		Q(j, 0) = q_corigin[j][0];
		Q(j, 1) = q_corigin[j][1];
		Q(j, 2) = q_corigin[j][2];
		P(j, 0) = p_corigin[j][0];
		P(j, 1) = p_corigin[j][1];
		P(j, 2) = p_corigin[j][2];		
	}
	Eigen::MatrixXf S = Q.transpose() * P;
	// 3. Compute the Singular Value Decomposition of S
	Eigen::JacobiSVD<Eigen::MatrixXf> eigensolver(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf U = eigensolver.matrixU();
	Eigen::MatrixXf V = eigensolver.matrixV();

	// 4. Compute the optimal rotation matrix using U and V
	Eigen::MatrixXf R = U * V.transpose();
	R.conservativeResizeLike(Eigen::MatrixXf::Identity(4,4));
	
	// 5. If reflection, cancel it
	if (S.determinant() == -1) {
		R(4, 4) *= -1;
	}
	// 6. Compute the traslation vector
	Eigen::Vector4f t = Eigen::Vector4f(p_centroid.x, p_centroid.y, p_centroid.z, 1) - R * Eigen::Vector4f(q_centroid.x, q_centroid.y, q_centroid.z, 1);
	
	return glm::mat4(R(0, 0), R(0, 1), R(0, 2), t(0),
					 R(1, 0), R(1, 1), R(1, 2), t(1),
					 R(2, 0), R(2, 1), R(2, 2), t(2),
					 R(3, 0), R(3, 1), R(3, 2), R(3,3));
}


// This method should perform the whole ICP algorithm with as many steps as needed.
// It should stop when maxSteps are performed, when the Frobenius norm of the transformation matrix of
// a step is smaller than a small threshold, or when the correspondence does not change from the 
// previous step.
bool IterativeClosestPoint::hasConverged(glm::mat4 matrix) {
	glm::mat3 R;
	R[0] = glm::vec3(matrix[0][0], matrix[0][1], matrix[0][2]);
	R[1] = glm::vec3(matrix[1][0], matrix[1][1], matrix[1][2]);
	R[2] = glm::vec3(matrix[2][0], matrix[2][1], matrix[2][2]);
	glm::vec3 t = glm::vec3(matrix[0][3], matrix[1][3], matrix[2][3]);
	glm::mat3 R_I = R - glm::mat3(1.f);
	float frobenius_norm = 0.f;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			frobenius_norm += R_I[i][j] * R_I[i][j];
	frobenius_norm = sqrt(frobenius_norm);
	return glm::length(t) < convergence_threshold && frobenius_norm < convergence_threshold;
}

vector<int> *IterativeClosestPoint::computeFullICP(unsigned int maxSteps)
{
	// TODO
	bool converged = false;
	unsigned int curStep = 0;
	vector<int>* cur_correspondence;
	while (!converged && curStep < maxSteps) {
		curStep++;
		// 1. For every point qi in Q we find closest pj in P
		cur_correspondence = computeCorrespondence();
		// 2. Find optimal rigid transformation
		glm::mat4 matrix = computeICPStep();
		// 3. Apply transformation R,t to the points in Q
		cloud2->transform(matrix);
		converged = hasConverged(matrix);
	}
	
	return cur_correspondence;
}





