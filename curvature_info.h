#pragma once
#include <nlohmann/json.hpp>
#include "type.h"


/**
 * @brief Curvature information
 */
struct CurvatureInfo
{
	ScalarArray mean;	///< mean curvature
	ScalarArray gaussian;	///< gaussian curvature
	ScalarArray principal_value1;	///< principal curvature value 1
	VectorArray principal_directions1;	///< principal curvature direction 1
	ScalarArray principal_value2;	///< principal curvature value 2
	VectorArray principal_directions2;	///< principal curvature direction 2
};


/**
* @brief Initialize curvature information
* @param curvature_info curvature information
*/
void Initialize(CurvatureInfo& curvature_info);


/**
 * @brief Calculate curvature information
 * @param V vertex array
 * @param F face array
 * @param curvature_info curvature information
 * @return void
 */
void CalcCurvatures(const VectorArray& V, const IndicesArray& F, CurvatureInfo& curvature_info);


// serialize functions
/**
 * @brief Convert CurvatureInfo to json
 * @param j json object
 * @param info curvature information
 * @return void
 */
void to_json(nlohmann::json& j, const CurvatureInfo& info);