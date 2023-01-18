#ifndef _SCENE_INCLUDE
#define _SCENE_INCLUDE


#include <glm/glm.hpp>
#include "Camera.h"
#include "ShaderProgram.h"
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

	void greyColors();
	void computeTexCoords();

private:
	Camera camera;
	TriangleMesh *mesh;
	ShaderProgram regularProgram, parameterSpaceProgram;
	float currentTime;
	
	bool bLighting, bParameterSpace;

};


#endif // _SCENE_INCLUDE

