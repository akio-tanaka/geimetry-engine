#pragma once
#include "type.h"


/**
* @brief Convert a vector to an Eigen vector
*        if the size of the vector is not 3, the function will raise an std::invalid_argument exception.
* @param v vector
*/
Eigen::Vector3d Convert(const std::vector<double>& v);


/**
* @brief Find the nearest vertex to a given coordinate
* @param V vertices
* @param F faces
* @param coordinate coordinate
* @return index of the nearest vertex
*/
int FindNearestVertex(const VectorArray& V, const IndicesArray& F, const Eigen::Vector3d& coordinate);