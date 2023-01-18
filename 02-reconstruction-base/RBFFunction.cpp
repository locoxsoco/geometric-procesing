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
	vector<glm::vec3> points_plus;
	vector<glm::vec3> points_minus;
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		points_plus.push_back(cloud->getPoints()[i] + d* cloud->getNormals()[i]);
		points_minus.push_back(cloud->getPoints()[i] - d* cloud->getNormals()[i]);
	}
	nn.setPoints(&(pointCloud->getPoints()));
	// Use gaussian RBFs with compact support for a sparse matrix
	Eigen::VectorXd c(cloud->getPoints().size() * 3); 
	Eigen::VectorXd v = Eigen::VectorXd::Zero(cloud->getPoints().size() * 3);
	Eigen::SparseMatrix<double> A(cloud->getPoints().size()*3, cloud->getPoints().size()*3);
	// Fill v
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		v(cloud->getPoints().size()+i) = (double)d;
		v(cloud->getPoints().size()*2+i) = -(double)d;
	}
	// Fill A_plus, A_minus
	// Most of the time is spent here, a possible improve may be by using Triplets
	// for batch insertion in sparse matrices
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		glm::vec3 pi;
		vector< pair<size_t, float> > neighbor_ids;
		// pi
		pi = cloud->getPoints()[i];
		nn.getNeighborsInRadius(pi, nn_radius, neighbor_ids);
		for (int j = 0; j < neighbor_ids.size(); j++) {
			glm::vec3 pj = cloud->getPoints()[neighbor_ids[j].first];
			double r = neighbor_ids[j].second;
			double rbf = gaussianRBF(r);
			A.insert(i, neighbor_ids[j].first) = rbf;
		}
		// pi_plus
		pi = points_plus[i];
		nn.getNeighborsInRadius(pi, nn_radius, neighbor_ids);
		for(int j=0; j<neighbor_ids.size();j++) {
			glm::vec3 pj = cloud->getPoints()[neighbor_ids[j].first];
			double r = neighbor_ids[j].second;
			double rbf = gaussianRBF(r);
			A.insert(cloud->getPoints().size() + i, neighbor_ids[j].first) = rbf;
			A.insert(neighbor_ids[j].first, cloud->getPoints().size() + i) = rbf;
		}
		// pi_minus
		pi = points_minus[i];
		nn.getNeighborsInRadius(pi, nn_radius, neighbor_ids);
		for (int j = 0; j < neighbor_ids.size(); j++) {
			glm::vec3 pj = cloud->getPoints()[neighbor_ids[j].first];
			double r = neighbor_ids[j].second;
			double rbf = gaussianRBF(r);
			A.insert(cloud->getPoints().size() * 2 + i, neighbor_ids[j].first) = rbf;
			A.insert(neighbor_ids[j].first, cloud->getPoints().size() * 2 + i) = rbf;
		}
	}
	// Matrix regularization A = A + lambda*I
	for (int i = 0; i < cloud->getPoints().size()*3; i++) {
		A.coeffRef(i, i) += lambda_regularization;
	}
	// Solve c
	Eigen::BiCGSTAB<Eigen::SparseMatrix<double> > solver_plus;
	solver_plus.compute(A);
	c = solver_plus.solve(v);
	for (int i = 0; i < cloud->getPoints().size(); i++) {
		vc.push_back(c[i]);
	}	
	return;
}

double RBFFunction::gaussianRBF(double r) const {
	return glm::exp(-r * r / (2.0 * (double)std_dv * (double)std_dv));
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
		float rbf_i = gaussianRBF(neighbor_ids[i].second);
		f_p += rbf_i * vc[neighbor_ids[i].first];
	}
	value = f_p;
	return true;
}






