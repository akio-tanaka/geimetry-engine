#pragma once
#include <filesystem>
#include "type.h"


struct CurvatureInfo;


/**
 * @brief Load a model file
 * @param path path to the model file
 * @param V vertices
 * @param F faces
 * @return true if the model is loaded successfully, false otherwise
 */
bool LoadModel(const std::filesystem::path& path, VectorArray& V, IndicesArray& F);


/**
 * @brief Save curvatures to a json file
 * @param filepath file path
 * @param info curvature information
 * @return true if the curvatures are saved successfully, false otherwise
 */
bool SaveCurvatures(const std::filesystem::path& filepath, const CurvatureInfo& info);


/**
 * @brief Save the model to a vtk file
 * @param filepath file path
 * @param V vertices
 * @param F faces
 * @param curvature curvature information
 * @return true if the vtk file is saved successfully, false otherwise
 */
bool SaveVtk(
	const std::filesystem::path& filepath,
	const VectorArray& V,
	const IndicesArray& F,
	const CurvatureInfo& curvature);


/**
 * @brief Save a polyline to a file
 * @param filepath file path
 * @param V vertices
 * @param selected selected vertices
 * @return true if the polyline is saved successfully, false otherwise
 */
bool SaveCsv(
	const std::filesystem::path& filepath,
	const VectorArray& V,
	const std::vector<int>& selected);


/**
 * @brief Save a polyline to a file
 * @param filepath file path
 * @param V vertices
 * @param selected selected vertices
 * @return true if the polyline is saved successfully, false otherwise
 */
bool SaveCsv(
	const std::filesystem::path& filepath,
	const VectorArray& V);