#include <iostream>
#include <algorithm>
#include "IterativeClosestPoint.h"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <glm/gtc/matrix_transform.hpp>


void IterativeClosestPoint::setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2)
{
	cloud1 = pointCloud1;
	cloud2 = pointCloud2;
}

// This method should mark the border points in cloud 1. It also changes their color (for example to red).
// You will need to add an attribute to this class that stores this property for all points in cloud 1. 

void IterativeClosestPoint::markBorderPoints(PointCloud &cloud)
{
	// TODO
	const vector<glm::vec3>* points1 = &(cloud1->getPoints());

	NearestNeighbors knn;
	knn.setPoints(points1);

	for (unsigned int i = 0; i < cloud1->getPoints().size(); i++) {
		glm::vec3 point = cloud1->getPoints()[i];
		// 1. Compute its k-nearest neighbors
		unsigned int kNeighbors = 8;
		vector<size_t> neighbor_ids;
		vector<float> dists_squared;
		knn.getKNearestNeighbors(point, kNeighbors, neighbor_ids, dists_squared);

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
		float max_delta_alpha_threshold = 10.f;
		int is_border_point = 0;
		// Looking for an adjacent difference > threshold
		for (unsigned int j = 0; j < neighbor_ids.size()-1; j++) {
			if (polar_representation_angles[j] - polar_representation_angles[j + 1] > max_delta_alpha_threshold) {
				is_border_point = 1;
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
	vector<int> correspondence;
	for (int i = 0; i < cloud2->getPoints().size(); i++) {
		if()
	}
	
	return NULL;
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
	glm::vec3 p_centroid = getCentroid(*points1);
	vector<glm::vec3> p_corigin = getPointsCentered(*points1, p_centroid);
	glm::vec3 q_centroid = getCentroid(*points2);
	vector<glm::vec3> q_corigin = getPointsCentered(*points2, q_centroid);
	// 2. Compute the covariance matrix S of both points sets
	Eigen::Matrix3f S;
	S = Eigen::Matrix3f::Zero();
	for (unsigned int j = 0; j < q_corigin.size(); j++) {
		S += Eigen::Vector3f(
			q_corigin[j][0],
			q_corigin[j][1],
			q_corigin[j][2]
		) * Eigen::Vector3f(
			p_corigin[j][0],
			p_corigin[j][1],
			p_corigin[j][2]
		).transpose();
	}
	// 3. Compute the Singular Value Decomposition of S
	Eigen::JacobiSVD<Eigen::Matrix3f> eigensolver(S);
	Eigen::Matrix4f U = eigensolver.matrixU();
	Eigen::Matrix4f V = eigensolver.matrixV();
	// 4. Compute the optimal rotation matrix using U and V
	Eigen::Matrix4f R = V * U.transpose();
	// 5. If reflection, cancel it
	if (S.determinant() == -1) {
		R(3, 3) = -1;
	}
	// 6. Compute the traslation vector
	//glm::vec4 t = p_centroid - R * q_centroid;
	
	return glm::mat4(1.f);
}


// This method should perform the whole ICP algorithm with as many steps as needed.
// It should stop when maxSteps are performed, when the Frobenius norm of the transformation matrix of
// a step is smaller than a small threshold, or when the correspondence does not change from the 
// previous step.

vector<int> *IterativeClosestPoint::computeFullICP(unsigned int maxSteps)
{
	// TODO
	// 1. For every point qi in Q we find closest pj in P
	// 2. Find optimal rigid transformation
	// 3. Apply transformation R,t to the points in Q

	


	bool converged = false;
	unsigned int curStep = 0;
	while (!converged && curStep < maxSteps) {
		glm::mat4 r = computeICPStep();
		//converged = hasConverged(r,t);
	}
	
	return NULL;
}





