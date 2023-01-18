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
		cycle_length += glm::length(mesh->getVertices()[cycle[i]] - mesh->getVertices()[cycle[i + 1]]);
	}
	cycle_length += glm::length(mesh->getVertices()[cycle[cycle.size() - 1]] - mesh->getVertices()[cycle[0]]);
	
	// We build an equation system to compute the harmonic coordinates of the 
	// interior vertices
	// TODO
	
	// Finally, we solve the system and assign the computed texture coordinates
	// to their corresponding vertices on the input mesh
	// TODO
	return;
}






