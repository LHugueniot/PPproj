#ifndef KERNEL_H
#define KERNEL_H

#include "point.h"
#include "indexDuo.h"
#include "constraint.h"

// glm dependencies
#include <glm/glm.hpp>

//Assimp dependencies
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace LuHu {

/// @param vector comprised positions

using posVector = std::vector<glm::vec3>;

/// @param vector comprised of point classes

using pVector = std::vector<point>;

/// @brief uses assimp to return a pointer to a scene containing models

const aiScene* getModel(std::string _model);

/// @brief gets all models in scene and stores points in a posVector

posVector storePoints(const aiScene* scene,uint meshIndex);

/// @brief creates new points based on a posVector

pVector posToPoint(posVector _pPoints);

/// @brief looks in bigVec and checks if temp exists in it

bool compare(glm::vec3 temp, posVector bigVec);

/// @brief cross compare two vectors

bool compare( posVector bigVec1,  posVector bigVec2);

/// @brief removes all duplicates from a posVector

posVector removeDuplicates(posVector _pVec);

/// @brief converts from assimmp vec3 format to glm::vec3

glm::vec3 aiToGLM(aiVector3D mVertices);

/// @brief convinient way to print vec3s

void printVec3(glm::vec3 _vec);

/// @brief gets an array of pointers to points of faces, to iterate through face positions go throught it 3 by 3

std::vector<std::shared_ptr<point>> getFaces(const aiScene* scene, uint meshIndex,
                                             std::vector<std::shared_ptr<point>> &pointVector, posVector defaultIndexing);

/// @brief returns all edges of a given mesh and stores it in an vector of indexDuo

std::vector<indexDuo> getEdges(const aiScene* scene,uint meshIndex, posVector defaultIndexing);

/// @brief creates distance constraints based on edges

std::vector<std::shared_ptr<constraint>> createDistanceConstraints(std::vector<indexDuo> edges,
                                                  std::vector<glm::vec3> vertices,
                                                  std::vector<std::shared_ptr<point> > &pointVector);

}
#endif // KERNEL_H
