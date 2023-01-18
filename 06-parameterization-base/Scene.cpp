#define NOMINMAX

#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <algorithm>
#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include "imgui/imgui.h"
#include "Scene.h"
#include "Parameterizer.h"


Scene::Scene()
{
	mesh = NULL;
}

Scene::~Scene()
{
	if(mesh != NULL)
	{
		mesh->free();
		delete mesh;
	}
}


void Scene::init()
{
	initShaders();

	currentTime = 0.0f;
	
	camera.init(2.0f);
	
	bLighting = true;
	bParameterSpace = false;
}

bool Scene::loadScan(const char *filename)
{
	ifstream fin;
	
	// Check if file exists
	fin.open(filename);
	if(!fin.is_open())
		return false;
	
	if(mesh != NULL)
		delete mesh;
	mesh = new TriangleMesh();
	if(mesh->load(filename))
	{
		cout << "Triangle mesh " << filename << " loaded. ";
		cout << mesh->getVertices().size() << " vertices. ";
		cout << (mesh->getTriangles().size()/3) << " faces." << endl;
	}
	else
		return false;

	// Compute global bounding box
	glm::vec3 bbox[2], center, size;
	glm::mat4 matrix;
	
	getBBox(bbox);
	center = (bbox[0] + bbox[1]) / 2.0f;
	size = bbox[1] - bbox[0];
	size = glm::vec3(glm::max(size.x, glm::max(size.y, size.z)));
	matrix = glm::mat4(1.0f);
	matrix = glm::scale(matrix, glm::vec3(1.0f - 0.1f, 1.0f - 0.1f, 1.0f - 0.1f) / size);
	matrix = glm::translate(matrix, -center);
	transform(matrix);
	
	greyColors();
	computeTexCoords();
	
	return true;
}

void Scene::update(int deltaTime)
{
	currentTime += deltaTime;
}

void Scene::render()
{
	if(bParameterSpace)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		parameterSpaceProgram.use();
		
		if(mesh != NULL)
			mesh->render(parameterSpaceProgram);
	}
	else
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		regularProgram.use();
		regularProgram.setUniform1i("bLighting", int(bLighting));
		regularProgram.setUniformMatrix4f("projection", camera.getProjectionMatrix());
		regularProgram.setUniformMatrix4f("modelview", camera.getModelViewMatrix());
		regularProgram.setUniformMatrix3f("normalMatrix", glm::inverseTranspose(glm::mat3(camera.getModelViewMatrix())));
		
		if(mesh != NULL)
			mesh->render(regularProgram);
	}
}

void Scene::render_gui()
{
	ImGui::Begin("Options", 0, ImGuiWindowFlags_AlwaysAutoResize);

	ImGui::Text("Render");
	ImGui::Spacing();
	ImGui::Checkbox("Lighting", &bLighting);
	ImGui::Spacing();
	ImGui::Checkbox("Parameter space", &bParameterSpace);
	ImGui::Spacing();

	ImGui::End();
}

Camera &Scene::getCamera()
{
	return camera;
}

void Scene::initShaders()
{
	Shader vShader, fShader;

	// Loading regular shader
	// Load shader for point rendering
	vShader.initFromFile(VERTEX_SHADER, "shaders/texture.vert");
	if(!vShader.isCompiled())
	{
		cout << "Vertex Shader Error" << endl;
		cout << "" << vShader.log() << endl << endl;
	}
	fShader.initFromFile(FRAGMENT_SHADER, "shaders/texture.frag");
	if(!fShader.isCompiled())
	{
		cout << "Fragment Shader Error" << endl;
		cout << "" << fShader.log() << endl << endl;
	}
	regularProgram.init();
	regularProgram.addShader(vShader);
	regularProgram.addShader(fShader);
	regularProgram.link();
	if(!regularProgram.isLinked())
	{
		cout << "Shader Linking Error" << endl;
		cout << "" << regularProgram.log() << endl << endl;
	}
	regularProgram.bindFragmentOutput("outColor");
	vShader.free();
	fShader.free();
	
	// Loading parameter space shader
	// Load shader for point rendering
	vShader.initFromFile(VERTEX_SHADER, "shaders/parameterSpace.vert");
	if(!vShader.isCompiled())
	{
		cout << "Vertex Shader Error" << endl;
		cout << "" << vShader.log() << endl << endl;
	}
	fShader.initFromFile(FRAGMENT_SHADER, "shaders/parameterSpace.frag");
	if(!fShader.isCompiled())
	{
		cout << "Fragment Shader Error" << endl;
		cout << "" << fShader.log() << endl << endl;
	}
	parameterSpaceProgram.init();
	parameterSpaceProgram.addShader(vShader);
	parameterSpaceProgram.addShader(fShader);
	parameterSpaceProgram.link();
	if(!parameterSpaceProgram.isLinked())
	{
		cout << "Shader Linking Error" << endl;
		cout << "" << parameterSpaceProgram.log() << endl << endl;
	}
	parameterSpaceProgram.bindFragmentOutput("outColor");
	vShader.free();
	fShader.free();
}

void Scene::getBBox(glm::vec3 bbox[2]) const
{
	mesh->getBBox(bbox);
}

void Scene::transform(const glm::mat4 &matrix)
{
	mesh->transform(matrix);
	mesh->sendToOpenGL();
}

void Scene::greyColors()
{
	const vector<glm::vec3> &vertices = mesh->getVertices();
	vector<glm::vec4> &colors = mesh->getColors();

	colors.clear();
	colors.resize(vertices.size(), glm::vec4(0.5f, 0.5f, 0.5f, 1.0f));
	mesh->sendToOpenGL();
}

void Scene::computeTexCoords()
{
	Parameterizer param;
	
	param.harmonicCoordinates(mesh);
	mesh->sendToOpenGL();
}











