#ifndef _SCENE_INCLUDE
#define _SCENE_INCLUDE


#include <glm/glm.hpp>
#include "Camera.h"
#include "ShaderProgram.h"
#include "PointCloud.h"
#include "TriangleMesh.h"


// Scene contains all the entities of our application.
// It is responsible for updating and rendering them.


class Scene
{

public:
	Scene();
	~Scene();

	void init();
	void update(int deltaTime);
	void render();
	
	bool loadScan(const char *filename);
	
	void render_gui();
	
	void getBBox(glm::vec3 bbox[2]) const;
	void transform(const glm::mat4 &matrix);

	Camera &getCamera();
	
	void switchPolygonMode();

private:
	void initShaders();
	void computeModelViewMatrix();
	void extractMesh();

private:
	Camera camera;
	PointCloud cloud;
	TriangleMesh *mesh;
	ShaderProgram pointsProgram;
	float currentTime;
	
	bool bLighting;
	bool bDrawPointCloud, bDrawMesh;
	int pointSize;
	int selectedFunction, dmcResolution;

};


#endif // _SCENE_INCLUDE

