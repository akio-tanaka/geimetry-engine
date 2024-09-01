#pragma once
#include "type.h"


/**
* @brief Chaikin smoothing
* @param V vertices
* @param loop loop
* @param num_iterations number of iterations
* @return smoothed vertices
*/
VectorArray ChaikinSmoothing(const VectorArray& V, const std::vector<int>& loop, int num_iterations);


VectorArray ChaikinSmoothing2(const VectorArray& V, const std::vector<int>& loop, int num_iterations);
