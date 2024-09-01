#pragma once
#include <set>
#include "type.h"


struct CurvatureInfo;


/**
 * @brief Traverse the mesh along the margin line
 * @param V [i] vertices
 * @param F [i] faces
 * @param adjacency_list [i] adjacency list
 * @param curvature_info [i] curvature information
 * @param marginline [i/o] marginline must have a seed point as input
 * @param visited [o] visited vertices
 * @return void
 */
void CreateMarginline(
	const VectorArray& V,
	const IndicesArray& F,
	const std::vector<std::vector<int>>& adjacency_list,
	const CurvatureInfo& curvature_info,
	std::vector<int>& marginline,
	std::set<int>& visited);


/**
* @brief Traverse the mesh along the margin line
* @param V [i] vertices
* @param F [i] faces
* @param adjacency_list [i] adjacency list
* @param curvature_info [i] curvature information
* @param marginline [i/o] marginline must have a seed point as input
* @param visited [o] visited vertices
* @param num_samples [i] number of samples
* @param threshold_to_remove_last_point [i] threshold to remove the last point
* @return downsampled marginline 
*/
std::vector<int> DownSampleMarginline(
	const VectorArray& V,
	const IndicesArray& F,
	const std::vector<std::vector<int>>& adjacency_list,
	const CurvatureInfo& curvature_info,
	const std::vector<int>& marginline,
	const std::set<int>& visited,
	size_t num_samples,
	double threshold_to_remove_last_point);