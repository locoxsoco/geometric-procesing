#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "LaplacianSmoothing.h"


void LaplacianSmoothing::setMesh(TriangleMesh *newMesh)
{
	mesh = newMesh;
}

/* This method should apply nIterations iterations of the laplacian vector multiplied by lambda 
   to each of the vertices. */

void LaplacianSmoothing::iterativeLaplacian(int nIterations, float lambda)
{
	int counter = 0;
	while (counter < nIterations) {
		counter++;
		// For each vertex
		for (unsigned int i = 0; i < mesh->getVertices().size(); i++) {
			glm::vec3 pi = mesh->getVertices()[i];
			// Get Ni that represents the 1-ring vertex neighbors
			vector<unsigned int> neighbors;
			mesh->getNeighbors(i, neighbors);
			glm::vec3 laplacian_pi = glm::vec3(0);
			for (unsigned j = 0; j < neighbors.size(); j++) {
				glm::vec3 pj = mesh->getVertices()[neighbors[j]];
				laplacian_pi += pj - pi;
			}
			laplacian_pi /= neighbors.size();
			glm::vec3 pi_prime = pi + lambda * laplacian_pi;
			mesh->getVertices()[i] = pi_prime;
		}
	}
}

/* This method should apply nIterations iterations of the bilaplacian operator using lambda 
   as a scaling factor. */

void LaplacianSmoothing::iterativeBilaplacian(int nIterations, float lambda)
{
	int counter = 0;
	while (counter < nIterations) {
		counter++;
		// For each vertex
		for (unsigned int i = 0; i < mesh->getVertices().size(); i++) {
			glm::vec3 pi = mesh->getVertices()[i];
			// Get Ni that represents the 1-ring vertex neighbors
			vector<unsigned int> neighbors;
			mesh->getNeighbors(i, neighbors);
			glm::vec3 laplacian_pi = glm::vec3(0);
			for (unsigned j = 0; j < neighbors.size(); j++) {
				glm::vec3 pj = mesh->getVertices()[neighbors[j]];
				laplacian_pi += pj - pi;
			}
			laplacian_pi /= neighbors.size();
			glm::vec3 pi_prime = pi + lambda * laplacian_pi;

			glm::vec3 laplacian_pi_prime = glm::vec3(0);
			for (unsigned j = 0; j < neighbors.size(); j++) {
				glm::vec3 pj = mesh->getVertices()[neighbors[j]];
				laplacian_pi_prime += pj - pi_prime;
			}
			laplacian_pi_prime /= neighbors.size();
			glm::vec3 pi_prime_prime = pi_prime - lambda * laplacian_pi_prime;
			
			mesh->getVertices()[i] = pi_prime_prime;
		}
	}
}

/* This method should apply nIterations iterations of Taubin's operator using lambda 
   as a scaling factor, and computing the corresponding nu value. */

void LaplacianSmoothing::iterativeLambdaNu(int nIterations, float lambda)
{
	int counter = 0;
	float mu = lambda / (0.1 * lambda - 1);
	while (counter < nIterations) {
		counter++;
		// For each vertex
		for (unsigned int i = 0; i < mesh->getVertices().size(); i++) {
			glm::vec3 pi = mesh->getVertices()[i];
			// Get Ni that represents the 1-ring vertex neighbors
			vector<unsigned int> neighbors;
			mesh->getNeighbors(i, neighbors);
			glm::vec3 laplacian_pi = glm::vec3(0);
			for (unsigned j = 0; j < neighbors.size(); j++) {
				glm::vec3 pj = mesh->getVertices()[neighbors[j]];
				laplacian_pi += pj - pi;
			}
			laplacian_pi /= neighbors.size();
			glm::vec3 pi_prime = pi + lambda * laplacian_pi;

			glm::vec3 laplacian_pi_prime = glm::vec3(0);
			for (unsigned j = 0; j < neighbors.size(); j++) {
				glm::vec3 pj = mesh->getVertices()[neighbors[j]];
				laplacian_pi_prime += pj - pi_prime;
			}
			laplacian_pi_prime /= neighbors.size();
			glm::vec3 pi_prime_prime = pi_prime + mu * laplacian_pi_prime;

			mesh->getVertices()[i] = pi_prime_prime;
		}
	}
}

/* This method should compute new vertices positions by making the laplacian zero, while
   maintaing the vertices marked as constraints fixed. */

void LaplacianSmoothing::globalLaplacian(const vector<bool> &constraints)
{
	int nVertices = mesh->getVertices().size();
	Eigen::SparseMatrix<double> L(nVertices, nVertices);
	Eigen::SparseMatrix<double> M(nVertices, nVertices);
	Eigen::SparseMatrix<double> C(nVertices, nVertices);
	vector<unsigned int> neighbors;
	for (unsigned int i = 0; i < nVertices; i++) {
		mesh->getNeighbors(i, neighbors);
		//Fill M
		M.insert(i, i) = neighbors.size();
		//Fill C
		for (unsigned j = 0; j < neighbors.size(); j++) {
			C.coeffRef(i, neighbors[j]) = 1;
		}
		C.coeffRef(i, i) = -neighbors.size();
	}
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper > solver;
	solver.compute(M);
	Eigen::SparseMatrix<double> I(nVertices, nVertices);
	I.setIdentity();
	Eigen::SparseMatrix<double> M_inv(nVertices, nVertices);
	M_inv = solver.solve(I);
	L = M_inv * C;
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nVertices,3);
	for (unsigned int i = 0; i < nVertices; i++) {
		if (constraints[i]) {
			L.row(i) *= 0;
			L.coeffRef(i, i) = 1;
			S(i,0) = mesh->getVertices()[i].x;
			S(i,1) = mesh->getVertices()[i].y;
			S(i,2) = mesh->getVertices()[i].z;
		}
	}
	solver.setTolerance(0.0001);
	solver.compute(L);
	Eigen::MatrixXd P_prime(nVertices,3);
	Eigen::MatrixXd Guess = Eigen::MatrixXd::Zero(nVertices, 3);
	for (unsigned int i = 0; i < nVertices; i++) {
		Guess(i, 0) = mesh->getVertices()[i].x;
		Guess(i, 1) = mesh->getVertices()[i].y;
		Guess(i, 2) = mesh->getVertices()[i].z;
	}
	P_prime = solver.solveWithGuess(S, Guess);
	for (unsigned int i = 0; i < nVertices; i++) {
		mesh->getVertices()[i] = glm::vec3(P_prime(i, 0), P_prime(i, 1), P_prime(i, 2));
	}	
}

/* This method has to optimize the vertices' positions in the least squares sense, 
   so that the laplacian is close to zero and the vertices remain close to their 
   original locations. The constraintWeight parameter is used to control how close 
   the vertices have to be to their original positions. */

void LaplacianSmoothing::globalBilaplacian(const vector<bool> &constraints, float constraintWeight)
{
	int nVertices = mesh->getVertices().size();
	int nConstraints = std::count(constraints.begin(), constraints.end(), true);
	Eigen::SparseMatrix<double> L(nVertices, nVertices);
	Eigen::SparseMatrix<double> M(nVertices, nVertices);
	Eigen::SparseMatrix<double> C(nVertices, nVertices);
	vector<unsigned int> neighbors;
	for (unsigned int i = 0; i < nVertices; i++) {
		mesh->getNeighbors(i, neighbors);
		//Fill M
		M.insert(i, i) = neighbors.size();
		//Fill C
		for (unsigned j = 0; j < neighbors.size(); j++) {
			C.coeffRef(i, neighbors[j]) = 1;
		}
		C.coeffRef(i, i) = -neighbors.size();
	}
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper > solver;
	solver.compute(M);
	Eigen::SparseMatrix<double> I(nVertices, nVertices);
	I.setIdentity();
	Eigen::SparseMatrix<double> M_inv(nVertices, nVertices);
	M_inv = solver.solve(I);
	L = M_inv * C;
	
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nVertices + nConstraints, 3);
	Eigen::SparseMatrix<double> L_new(nVertices + nConstraints, nVertices);
	Eigen::MatrixXd matL(nVertices + nConstraints, nVertices);
	matL <<
		Eigen::MatrixXd(L),
		Eigen::MatrixXd::Zero(nConstraints, nVertices);
	L_new = matL.sparseView();
	int constraint_counter = 0;
	for (unsigned int i = 0; i < nVertices; i++) {
		if (constraints[i]) {
			L_new.insert(nVertices + constraint_counter, nVertices - nConstraints + constraint_counter) = 1 * constraintWeight;
			S(nVertices + constraint_counter, 0) = mesh->getVertices()[i].x * constraintWeight;
			S(nVertices + constraint_counter, 1) = mesh->getVertices()[i].y * constraintWeight;
			S(nVertices + constraint_counter, 2) = mesh->getVertices()[i].z * constraintWeight;
			constraint_counter++;
		}
	}
	Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double> > lscg;
	solver.setTolerance(0.0001);
	lscg.compute(L_new);
	Eigen::MatrixXd P_prime(nVertices, 3);
	Eigen::MatrixXd Guess = Eigen::MatrixXd::Zero(nVertices, 3);
	for (unsigned int i = 0; i < nVertices; i++) {
		Guess(i, 0) = mesh->getVertices()[i].x;
		Guess(i, 1) = mesh->getVertices()[i].y;
		Guess(i, 2) = mesh->getVertices()[i].z;
	}
	P_prime = lscg.solveWithGuess(S, Guess);
	for (unsigned int i = 0; i < nVertices; i++) {
		mesh->getVertices()[i] = glm::vec3(P_prime(i, 0), P_prime(i, 1), P_prime(i, 2));
	}
}








