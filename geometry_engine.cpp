#include "geometry_engine.h"
#include <iostream>
#include <fstream>
#include <igl/adjacency_list.h>
#include "geometry_utils.h"
#include "return_code.h"
#include "io_utils.h"
#include "marginline.h"


namespace
{
	std::vector<std::vector<double>> Convert(const VectorArray& V, const std::vector<int>& indices)
	{
		std::vector<std::vector<double>> result;
		for (auto i : indices)
		{
			result.push_back({ V(i, 0), V(i, 1), V(i, 2) });
		}
		return result;
	}


	std::filesystem::path GetOutputPath(const std::filesystem::path& input_json)
	{
		std::filesystem::path output_path = input_json.parent_path() / "output.json";
		return output_path;
	}


	bool SaveOutput(const std::filesystem::path& filepath, const GeometryEngineOutput& output)
	{
		try
		{
			std::ofstream out(filepath);
			nlohmann::json json_obj = output;
			out << json_obj;
			return true;
		}
		catch (std::exception& e)
		{
			std::cout << "failed to save the output file\n";
			std::cout << e.what() << "\n";
			return false;
		}
	}
}


bool GeometryEngine::Initialize(const std::filesystem::path& input_json)
{
	is_initialized_ = false;
	input_json_ = input_json;
	adjacency_list_.clear();
	::Initialize(curvature_info_);

	try
	{
		::Initialize(output_);
		std::ifstream ifs(input_json);
		if (!ifs.is_open())
		{
			output_.return_code = ToInt(ReturnCode::kInvalidInput);
			output_.message = "failed to open input json: " + input_json.string();

			SaveOutput(GetOutputPath(input_json_), output_);

			return false;
		}
		input_ = nlohmann::json::parse(ifs);
		ifs.close();
		std::cout << "input json is loaded\n";

		std::stringstream model_filename;
		model_filename << "model" << input_.model.type;
		auto filepath = input_json.parent_path() / model_filename.str();
		if (!LoadModel(filepath, V_, F_))
		{
			output_.return_code = ToInt(ReturnCode::kInvalidInput);
			output_.message = "failed to open model: " + filepath.string();

			SaveOutput(GetOutputPath(input_json_), output_);

			return false;
		}
		std::cout << "model is loaded\n";

		igl::adjacency_list(F_, adjacency_list_);
		std::cout << "adjacency list is created\n";

		std::cout << "done to initialize geometry engine\n";

		is_initialized_ = true;
		return true;
	}
	catch (const std::exception& e)
	{
		output_.return_code = ToInt(ReturnCode::kUnknownError);
		output_.message = e.what();

		SaveOutput(GetOutputPath(input_json_), output_);

		std::cout << "failed to initialize geometry engine\n";
		return false;
	}
}



GeometryEngineOutput GeometryEngine::Run()
{
	::Initialize(output_);

	if (!is_initialized_)
	{
		output_.return_code = ToInt(ReturnCode::kInvalidModel);
		output_.message = " is not initialized";

		SaveOutput(GetOutputPath(input_json_), output_);

		return output_;
	}

	if (input_.operation.type != "marginline")
	{
		output_.return_code = ToInt(ReturnCode::kInvalidInput);
		output_.message = "invalid operation type: " + input_.operation.type + " (expected: marginline)";

		SaveOutput(GetOutputPath(input_json_), output_);

		return output_;
	}

	try
	{
		CalcCurvatures(V_, F_, curvature_info_);
		auto minH = curvature_info_.mean.minCoeff();
		auto maxH = curvature_info_.mean.maxCoeff();
		std::cout << "done to calculate curvatures\n";

#ifdef _DEBUG
		std::cout << "minH: " << minH << "\n";
		std::cout << "maxH: " << maxH << "\n";

		auto json_filepath = input_json_.parent_path() / "curvatures.json";
		if (!SaveCurvatures(json_filepath, curvature_info_))
		{
			std::cout << "failed to save curvatures\n";
		}

		auto vtk_filepath = std::filesystem::path(input_json_).replace_extension(".vtk");
		if (!SaveVtk(vtk_filepath, V_, F_, curvature_info_))
		{
			std::cout << "failed to save vtk file\n";
		}
#endif

		auto seed = Convert(input_.operation.marginline.seed);
		auto nearest_vertex = FindNearestVertex(V_, F_, seed);

		std::vector<int> marginline{ nearest_vertex };
		std::set<int> visited;
		CreateMarginline(V_, F_, adjacency_list_, curvature_info_, marginline, visited);

		auto num_samples = input_.operation.marginline.num_samples;
		auto threshold_to_remove_last_point = input_.operation.marginline.threshold_to_remove_last_point;
		auto downsampled = DownSampleMarginline(V_, F_, adjacency_list_, curvature_info_, marginline, visited, num_samples, threshold_to_remove_last_point);
		
		output_.result.type = "marginline";
		output_.result.marginline.num_original_points = marginline.size();
		output_.result.marginline.num_samples = downsampled.size();
		output_.result.marginline.points = Convert(V_, downsampled);
	}
	catch (const std::exception& e)
	{
		output_.return_code = ToInt(ReturnCode::kUnknownError);
		output_.message = e.what();
	}

	SaveOutput(GetOutputPath(input_json_), output_);
	return output_;
}