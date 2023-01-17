#include <iostream>
#include "MongePatch.h"


// Given a point P, its normal, and its closest neighbors (including itself) 
// compute a quadratic Monge patch that approximates the neighborhood of P.
// The resulting patch will be used to compute the principal curvatures of the 
// surface at point P.

void MongePatch::init(const glm::vec3 &P, const glm::vec3 &normal, const vector<glm::vec3> &closest)
{
	vertex = P;
	vs.clear();
	// First we need to transform the neighbors to a coordinate system (u, v, w)
	// aligned with the normal n.The origin will be point p itself
	glm::vec3 w = -normal;
	w = glm::normalize(w);
	glm::vec3 u = glm::cross(glm::vec3(1.0,0.0,0.0),w);
	u = glm::normalize(u);
	glm::vec3 v = glm::cross(w,u);
	v = glm::normalize(v);

	// We transform all the neighbors to this system from pi to (ui, vi, wi)
	vector<glm::vec3> pis_new_system(closest.size());
	for (int i = 0; i < closest.size(); i++) {
		pis_new_system[i] = glm::vec3(
			glm::dot(u, closest[i] - P),
			glm::dot(v, closest[i] - P),
			glm::dot(w, closest[i] - P)
		);
	}

	// Then we fit the function using least squares giving eq: As=b
	// where A is sum q_i * q_i^T, and b is suim w_i * q_i
	Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6,6);
	Eigen::VectorXf s;
	Eigen::VectorXf b = Eigen::VectorXf::Zero(6);
	Eigen::VectorXf q_i(6);
	for (int i = 0; i < closest.size(); i++) {
		q_i(0) = pis_new_system[i].x * pis_new_system[i].x;
		q_i(1) = pis_new_system[i].x * pis_new_system[i].y;
		q_i(2) = pis_new_system[i].y * pis_new_system[i].y;
		q_i(3) = pis_new_system[i].x;
		q_i(4) = pis_new_system[i].y;
		q_i(5) = 1;

		A += q_i * q_i.transpose();
		b += pis_new_system[i].z * q_i;
	}
	Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(A);
	s = dec.solve(b);

	for (int i = 0; i < 6; i++) {
		vs.push_back(s[i]);
	}
	return;
}

// Return the values of the two principal curvatures for this patch

void MongePatch::principalCurvatures(float &kmin, float &kmax) const
{
	Eigen::Matrix2f Hw;
	Hw(0, 0) = 2*vs[0];
	Hw(0, 1) = vs[1];
	Hw(1, 0) = vs[1];
	Hw(1, 1) = 2*vs[2];

	Eigen::JacobiSVD<Eigen::Matrix2f, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(Hw);
	Eigen::Vector2f pc = svd.singularValues();

	kmin = pc[0];
	kmax = pc[1];
}


