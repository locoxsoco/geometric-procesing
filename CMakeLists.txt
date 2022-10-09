cmake_minimum_required(VERSION 2.8)

cmake_policy(SET CMP0015 NEW)
cmake_policy(SET CMP0072 OLD)

project(01-icp)

set (CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  #set(CMAKE_BUILD_TYPE Release)
endif()

#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

execute_process(COMMAND ln -s ../shaders)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR "OPENGL not found!")
endif(NOT OPENGL_FOUND)

# GLUT
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
if(NOT GLUT_FOUND)
    message(ERROR "GLUT found!")
endif(NOT GLUT_FOUND)

# GLM
set (GLM_INCLUDE_DIR "C:/Program Files/glm")
find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIR})
if(NOT GLM_FOUND)
    message(ERROR "GLM not found!")
endif(NOT GLM_FOUND)

include_directories("${CMAKE_SOURCE_DIR}/gl3w")
include_directories("${CMAKE_SOURCE_DIR}/eigen")
include_directories("${CMAKE_SOURCE_DIR}/nanoflann")

#link_libraries(dl)
LINK_DIRECTORIES(
    ${appName} "${CMAKE_SOURCE_DIR}/freeglut/lib"
)

set(imgui_files imgui/imconfig.h imgui/imgui.cpp imgui/imgui_demo.cpp imgui/imgui_draw.cpp imgui/imgui.h imgui/imgui_internal.h imgui/imgui_widgets.cpp imgui/imstb_rectpack.h imgui/imstb_textedit.h imgui/imstb_truetype.h imgui/imgui_impl_opengl3.h imgui/imgui_impl_opengl3.cpp imgui/imgui_impl_glut.h imgui/imgui_impl_glut.cpp)
set(gl3w_files gl3w/gl3w.c gl3w/GL/gl3w.h gl3w/KHR/khrplatform.h)
set(nanoflann_files nanoflann/nanoflann.hpp)

set(appName 01-icp)
add_executable(${appName} ${imgui_files} ${gl3w_files} ${nanoflann_files} PointCloud.h PointCloud.cpp Camera.h Camera.cpp Scene.h Scene.cpp Shader.h Shader.cpp ShaderProgram.h ShaderProgram.cpp Application.h Application.cpp main.cpp NormalEstimator.h NormalEstimator.cpp NearestNeighbors.h NearestNeighbors.cpp IterativeClosestPoint.h IterativeClosestPoint.cpp SegmentCloud.h SegmentCloud.cpp)
target_link_libraries(${appName} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
