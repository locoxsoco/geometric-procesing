#include <iostream>
#include <cstdlib>
#include <set>
#include <map>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "Parameterizer.h"
#include "timing.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;


// This method should compute new texture coordinates for the input mesh
// using the harmonic coordinates approach with uniform weights.
// The 'TriangleMesh' class has a method 'getTexCoords' that may be used 
// to access and update its texture coordinates per vertex.

void Parameterizer::harmonicCoordinates(TriangleMesh *mesh)
{
	// First we need to determine the border edges of the input mesh
	// TODO
	int nVertices = mesh->getVertices().size();
	set<pair<int, int>> edges;
	int nTriangles = mesh->getTriangles().size();
	for (int i = 0; i < nTriangles; i+=3) {
		int first_t = mesh->getTriangles()[i];
		int second_t = mesh->getTriangles()[i+1];
		auto it_1 = std::find_if(edges.begin(), edges.end(), [first_t, second_t](const pair<int, int>& p) { return p.first == first_t && p.second == second_t; });
		auto it_2 = std::find_if(edges.begin(), edges.end(), [first_t, second_t](const pair<int, int>& p) { return p.first == second_t && p.second == first_t; });
		if (it_1 != edges.end()) {
			edges.erase(it_1);
		}
		else if(it_2 != edges.end()) {			
			edges.erase(it_2);
		}
		else {
			edges.insert(pair<int, int>(first_t, second_t));
		}

		first_t = mesh->getTriangles()[i+1];
		second_t = mesh->getTriangles()[i+2];
		it_1 = std::find_if(edges.begin(), edges.end(), [first_t, second_t](const pair<int, int>& p) { return p.first == first_t && p.second == second_t; });
		it_2 = std::find_if(edges.begin(), edges.end(), [first_t, second_t](const pair<int, int>& p) { return p.first == second_t && p.second == first_t; });
		if (it_1 != edges.end()) {
			edges.erase(it_1);
		}
		else if (it_2 != edges.end()) {
			edges.erase(it_2);
		}
		else {
			edges.insert(pair<int, int>(first_t, second_t));
		}

		first_t = mesh->getTriangles()[i+2];
		second_t = mesh->getTriangles()[i];
		it_1 = std::find_if(edges.begin(), edges.end(), [first_t, second_t](const pair<int, int>& p) { return p.first == first_t && p.second == second_t; });
		it_2 = std::find_if(edges.begin(), edges.end(), [first_t, second_t](const pair<int, int>& p) { return p.first == second_t && p.second == first_t; });
		if (it_1 != edges.end()) {
			edges.erase(it_1);
		}
		else if (it_2 != edges.end()) {
			edges.erase(it_2);
		}
		else {
			edges.insert(pair<int, int>(first_t, second_t));
		}
	}
	
	// Then, these edges need to be arranged into a single cycle, the border polyline
	// TODO
	vector<int> cycle;
	auto iter = edges.begin();
	while (iter != edges.end()) {
		int cur_first_t = iter->first;
		int cur_second_t = iter->second;
		cycle.push_back(iter->first);
		edges.erase(iter);
		iter = std::find_if(edges.begin(), edges.end(), [cur_second_t](const pair<int, int>& p) { return p.first == cur_second_t; });
	}

	// Each of the vertices on the border polyline will receive a texture coordinate
	// on the border of the parameter space ([0,0] -> [1,0] -> [1,1] -> [0,1]), using 
	// the chord length approach
	// TODO
	double cycle_length = 0;
	for (int i = 0; i < cycle.size()-1; i++) {
		cycle_length += glm::length(mesh->getVertices()[cycle[i+1]] - mesh->getVertices()[cycle[i]]);
	}
	cycle_length += glm::length(mesh->getVertices()[cycle[cycle.size() - 1]] - mesh->getVertices()[cycle[0]]);
	double cur_cycle_length = 0;
	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(nVertices, 2);
	for (int i = 0; i < cycle.size()-1; i++) {
		double cur_length = glm::length(mesh->getVertices()[cycle[i+1]] - mesh->getVertices()[cycle[i]]);
		cur_cycle_length += cur_length;
		if (cur_cycle_length < cycle_length / 4.0) {
			T(cycle[i+1], 0) = cur_cycle_length * 4.0/ cycle_length;
			T(cycle[i+1], 1) = 0.0;
		} else if (cur_cycle_length < cycle_length / 2.0) {
			T(cycle[i+1], 0) = 1.0;
			T(cycle[i+1], 1) = (cur_cycle_length - 1.0 / 4.0 * cycle_length) * 4.0 / cycle_length;
		} else if (cur_cycle_length < 3.0 * cycle_length / 4.0) {
			T(cycle[i+1], 0) = 1.0 - (cur_cycle_length - cycle_length / 2.0) * 4.0 / cycle_length;
			T(cycle[i+1], 1) = 1.0;
		} else {
			T(cycle[i+1], 0) = 0.0;
			T(cycle[i+1], 1) = 1.0 - (cur_cycle_length - 3.0 / 4.0 * cycle_length) * 4.0 / cycle_length;
		}
	}
	T(cycle[0], 0) = 0.0;
	T(cycle[0], 1) = 0.0;
	
	// We build an equation system to compute the harmonic coordinates of the 
	// interior vertices
	// TODO
	Eigen::SparseMatrix<double> L(nVertices, nVertices);
	Eigen::SparseMatrix<double> M_inv(nVertices, nVertices);
	Eigen::SparseMatrix<double> C(nVertices, nVertices);
	vector<unsigned int> neighbors;
	for (unsigned int i = 0; i < nVertices; i++) {
		mesh->getNeighbors(i, neighbors);
		//Fill M
		M_inv.insert(i, i) = 1.0 / neighbors.size();
		//Fill C
		for (unsigned j = 0; j < neighbors.size(); j++) {
			C.coeffRef(i, neighbors[j]) = 1.0;
		}
		C.coeffRef(i, i) = -(double)neighbors.size();
	}
	
	L = M_inv * C;
	for (int i = 0; i < cycle.size() - 1; i++) {
		L.row(cycle[i]) *= 0;
		L.coeffRef(cycle[i], cycle[i]) = 1.0;
	}
	L.prune(0, 0);
	
	// Finally, we solve the system and assign the computed texture coordinates
	// to their corresponding vertices on the input mesh
	// TODO
	Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double> > solver;
	solver.compute(L);
	Eigen::MatrixXd T_prime(nVertices, 2);
	T_prime = solver.solve(T);
	for (unsigned int i = 0; i < nVertices; i++) {
		mesh->getTexCoords()[i] = glm::vec2(T_prime(i, 0), T_prime(i, 1));
	}
	return;
}






