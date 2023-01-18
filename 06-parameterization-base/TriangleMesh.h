#ifndef _TRIANGLE_MESH_INCLUDE
#define _TRIANGLE_MESH_INCLUDE


#include <vector>
#include <glm/glm.hpp>
#include "ShaderProgram.h"


class TriangleMesh
{
public:
	TriangleMesh();

	void init(const vector<glm::vec3> &newVertices, const vector<unsigned int> &newTriangles);
	bool load(const string &filename);

	void getBBox(glm::vec3 box[2]) const;
	void transform(const glm::mat4 &matrix);

	void sendToOpenGL();
	void render(ShaderProgram &program) const;
	void free();
	
	const vector<glm::vec3> &getVertices() const { return vertices; }
	vector<glm::vec3> &getVertices() { return vertices; }
	const vector<unsigned int> &getTriangles() const { return triangles; }
	vector<unsigned int> &getTriangles() { return triangles; }
	const vector<glm::vec4> &getColors() const { return colors; }
	vector<glm::vec4> &getColors() { return colors; }
	const vector<glm::vec2> &getTexCoords() const { return texCoords; }
	vector<glm::vec2> &getTexCoords() { return texCoords; }

	const vector<glm::vec3> &getPerVertexNormals() { return perVertexNormals; }
	
	void getNeighbors(unsigned int vrtxId, vector<unsigned int> &neighbors) const;
	
	static int next(int corner);
	static int previous(int corner);

private:
	void computePerVertexNormals();
	void computeCornerTable();
	void setPlanarTexCoords();

private:
	vector<glm::vec3> vertices, perVertexNormals;
	vector<unsigned int> triangles, vertex2Corner;
	vector<glm::vec4> colors;
	vector<glm::vec2> texCoords;
	vector<int> oTable;

	bool bGLObjsInit;
	GLuint vao;
	GLuint vboPosition, vboNormal, vboColor, vboTexCoord;

};


#endif // _TRIANGLE_MESH_INCLUDE



