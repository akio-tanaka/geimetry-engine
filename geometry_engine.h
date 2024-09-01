#pragma once
#include <filesystem>
#include "input.h"
#include "output.h"
#include "curvature_info.h"



class GeometryEngine
{
	bool is_initialized_ = false;
	std::filesystem::path input_json_;

	GeometryEngineInput input_;
	GeometryEngineOutput output_;
	VectorArray V_;
	VectorArray N_;
	IndicesArray F_;
	std::vector<std::vector<int> > adjacency_list_;

	// curvature info
	CurvatureInfo curvature_info_;

public:
	GeometryEngine() = default;
	~GeometryEngine() = default;

	const GeometryEngineOutput& output() const { return output_; }
	VectorArray& V() { return V_; }
	VectorArray& N() { return N_; }
	IndicesArray& F() { return F_; }
	std::vector<std::vector<int> >& adjacency_list() { return adjacency_list_; }
	const CurvatureInfo& curvature_info() const { return curvature_info_; }

	bool Initialize(const std::filesystem::path& input_json);
	GeometryEngineOutput Run();
};
