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
	//knn.setPoints(&(cloud1->getPoints()));
}

// This method should mark the border points in cloud 1. It also changes their color (for example to red).
// You will need to add an attribute to this class that stores this property for all points in cloud 1. 

void IterativeClosestPoint::markBorderPoints()
{
	// TODO
	const vector<glm::vec3>* points1 = &(cloud1->getPoints());

	NearestNeighbors knn;
	knn.setPoints(points1);

	//for (glm::vec3 point : cloud1->getPoints()) {
	for (unsigned int i = 0; i < cloud1->getPoints().size(); i++) {
		glm::vec3 point = cloud1->getPoints()[i];
		// 1. Compute its k-nearest neighbors
		unsigned int kNeighbors = 8;
		vector<size_t> neighbor_ids;
		vector<float> dists_squared;
		knn.getKNearestNeighbors(point, kNeighbors, neighbor_ids, dists_squared);

		// 2. Use PCA to compute local reference frame for point p
		Eigen::Matrix3f eigenvectors = pca(*points1, neighbor_ids, dists_squared);

		// 3. Transform the k-nearest neighbors to this frame
		vector<glm::vec3> neighbors_frame(neighbor_ids.size());
		for (unsigned int j = 0; j < neighbor_ids.size(); j++) {
			glm::vec3 pj_pi = points1->at(neighbor_ids[j]) - point;
			neighbors_frame[j] = glm::vec3(
				glm::dot(pj_pi, glm::vec3(eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0))),
				glm::dot(pj_pi, glm::vec3(eigenvectors(0, 1), eigenvectors(1, 1), eigenvectors(2, 1))),
				glm::dot(pj_pi, glm::vec3(eigenvectors(0, 2), eigenvectors(1, 2), eigenvectors(2, 2)))
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
		// Looking for the right adjacent
		for (unsigned int j = 0; j < neighbor_ids.size()-1; j++) {
			if (polar_representation_angles[j] - polar_representation_angles[j + 1] > max_delta_alpha_threshold) {
				is_border_point = 1;
			}
		}
		// Looking for the left adjacent
		for (unsigned int j = 1; j < neighbor_ids.size(); j++) {
			if (polar_representation_angles[j] - polar_representation_angles[j - 1] > max_delta_alpha_threshold) {
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
	// 1. First compute the centroid corrected versions of the point sets
	glm::vec3 p, q;
	
	return NULL;
}


// This method should compute the rotation and translation of an ICP step from the correspondence
// information between clouds 1 and 2. Both should be encoded in the returned 4x4 matrix.
// To do this use the SVD algorithm in Eigen.

glm::mat4 IterativeClosestPoint::computeICPStep()
{
	// TODO
	// 1. For every point qi in Q we find closest pj in P
	// 2. Find optimal rigid transformation
	// 3. Apply transformation R,t to the points in Q
	
	return glm::mat4(1.f);
}


// This method should perform the whole ICP algorithm with as many steps as needed.
// It should stop when maxSteps are performed, when the Frobenius norm of the transformation matrix of
// a step is smaller than a small threshold, or when the correspondence does not change from the 
// previous step.

vector<int> *IterativeClosestPoint::computeFullICP(unsigned int maxSteps)
{
	// TODO
	const vector<glm::vec3>* points1 = &(cloud1->getPoints());
	const vector<glm::vec3>* points2 = &(cloud2->getPoints());

	// 1. Compute the centroids of the point sets
	// 2. Compute the covariance matrix S of both points sets
	// 3. Compute the Singular Value Decomposition of S
	// 4. Compute the optimal rotation matrix using U and V
	// 5. If reflection, cancel it
	// 6. Compute the traslation vector
	bool converged = false;
	unsigned int curStep = 0;
	while (!converged && curStep < maxSteps) {
		glm::mat4 r = computeICPStep();
		//converged = hasConverged(r,t);
	}
	
	return NULL;
}





