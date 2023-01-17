#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "RBFFunction.h"

/* Initialize everything to be able to compute the implicit distance to the reconstructed
   point cloud at arbitrary points that are close enough to the point cloud. As should be
   obvious by the name of the class, the distance has to be computed using RBFs.
 */

void RBFFunction::init(const PointCloud *pointCloud, float standardDeviation, float supportRadius)
{
	cloud = pointCloud;
	std_dv = standardDeviation;
	nn_radius = supportRadius;
	d *= supportRadius;
	// Create artificial points pi_plus, pi_minus
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		points_plus.push_back(cloud->getPoints()[i] + d* cloud->getNormals()[i]);
		points_minus.push_back(cloud->getPoints()[i] - d* cloud->getNormals()[i]);
	}
	nn.setPoints(&(pointCloud->getPoints()));
	// Use gaussian RBFs with compact support for a sparse matrix
	Eigen::VectorXf c_plus(cloud->getPoints().size()), v_plus(cloud->getPoints().size());
	Eigen::SparseMatrix<float> A_plus(cloud->getPoints().size(), cloud->getPoints().size());
	Eigen::VectorXf c_minus(cloud->getPoints().size()),v_minus(cloud->getPoints().size());
	Eigen::SparseMatrix<float> A_minus(cloud->getPoints().size(), cloud->getPoints().size());
	// Fill v_plus, v_minus
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		v_plus(i) = d;
		v_minus(i) = -d;
	}
	// Fill A_plus, A_minus
	// Most of the time is spent here, a possible improve may be by using Triplets
	// for batch insertion in sparse matrices
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		glm::vec3 pi;
		vector< pair<size_t, float> > neighbor_ids;
		// A_plus
		pi = points_plus[i];
		nn.getNeighborsInRadius(pi, nn_radius, neighbor_ids);
		for(int j=0; j<neighbor_ids.size();j++) {
			glm::vec3 pj = cloud->getPoints()[neighbor_ids[j].first];
			float r = glm::length(pi - pj);
			float rbf = gaussianRBF(r);
			A_plus.coeffRef(i, neighbor_ids[j].first) = rbf;
			A_plus.coeffRef(neighbor_ids[j].first,i) = rbf;
		}
		// A_minus
		pi = points_minus[i];
		nn.getNeighborsInRadius(pi, nn_radius, neighbor_ids);
		for (int j = 0; j < neighbor_ids.size(); j++) {
			glm::vec3 pj = cloud->getPoints()[neighbor_ids[j].first];
			float r = glm::length(pi - pj);
			float rbf = gaussianRBF(r);
			A_plus.coeffRef(i, neighbor_ids[j].first) = rbf;
			A_plus.coeffRef(neighbor_ids[j].first, i) = rbf;
		}
	}
	// Matrix regularization A = A + lambda*I
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		A_plus.coeffRef(i, i) += lambda_regularization;
		A_minus.coeffRef(i, i) += lambda_regularization;
	}
	// Solve c_plus, c_minus
	Eigen::BiCGSTAB<Eigen::SparseMatrix<float> > solver_plus;
	solver_plus.compute(A_plus);
	c_plus = solver_plus.solve(v_plus);
	//Eigen::BiCGSTAB<Eigen::SparseMatrix<float> > solver_minus;
	solver_plus.compute(A_minus);
	c_minus = solver_plus.solve(v_minus);
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		vc_plus.push_back(c_plus[i]);
		vc_minus.push_back(c_minus[i]);
	}

	
	
	return;
}

float RBFFunction::gaussianRBF(float r) const {
	return glm::exp(-r * r / (2 * std_dv * std_dv));
}

/* This operator returns a boolean that if true signals that the value parameter
   has been modified to contain the value of the RBF implicit distance at point P.
 */

bool RBFFunction::operator()(const glm::vec3 &P, float &value) const
{
	vector< pair<size_t, float> > neighbor_ids;
	nn.getNeighborsInRadius(P, nn_radius, neighbor_ids);
	if (neighbor_ids.size() == 0) return false;
	float f_p = 0.f;
	for (int i = 0; i < neighbor_ids.size(); i++) {
		glm::vec3 pi = cloud->getPoints()[neighbor_ids[i].first];
		float rbf_i = gaussianRBF(glm::length(P-pi));
		f_p += rbf_i * vc_plus[neighbor_ids[i].first];
		f_p += rbf_i * vc_minus[neighbor_ids[i].first];
	}
	f_p /= 2;
	return true;
}






