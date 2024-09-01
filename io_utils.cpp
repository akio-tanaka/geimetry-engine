#include "io_utils.h"
#include <iostream>
#include <igl/readPLY.h>
#include <igl/readSTL.h>
#include "curvature_info.h"



bool LoadModel(const std::filesystem::path& path, VectorArray& V, IndicesArray& F)
{
	auto get_extension = [](const std::filesystem::path& path) -> std::string
		{
			auto ext = path.extension();
			if (ext.empty())
			{
				return "";
			}

			auto ext_str = path.extension().string();
			std::transform(ext_str.begin(), ext_str.end(), ext_str.begin(), [](unsigned char c) { return std::tolower(c); });
			return ext_str;
		};

	try
	{
		auto extenstion = get_extension(path);
		if (extenstion == ".ply")
		{
			return igl::readPLY(path.string(), V, F);
		}
		else if (extenstion == ".stl")
		{
			std::ifstream in(path.c_str());
			Eigen::MatrixXf Vf, Nf;
			auto status = igl::readSTL(in, Vf, F, Nf);
			V = Vf.cast<double>();
			return status;
		}
		else
		{
			std::cout << "unsupported file format\n";
			return false;
		}
	}
	catch (std::exception& e)
	{
		std::cout << "failed to read the model file\n";
		std::cout << e.what() << "\n";
		return false;
	}
}


bool SaveCurvatures(const std::filesystem::path& filepath, const CurvatureInfo& info)
{
	try
	{
		std::ofstream out(filepath.c_str());
		nlohmann::json json_obj;
		to_json(json_obj, info);
		out << json_obj << std::endl;
		out.close();
		return true;
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << "\n";
		return false;
	}
}


bool SaveVtk(
	const std::filesystem::path& filepath,
	const VectorArray& V,
	const IndicesArray& F,
	const CurvatureInfo& curvature)
{
	std::ofstream out(filepath.c_str());
	if (!out)
	{
		return false;
	}

	out << "# vtk DataFile Version 2.0\n";
	out << "Unstructured Grid Example\n";
	out << "ASCII\n";
	out << "DATASET UNSTRUCTURED_GRID\n";
	out << "POINTS " << V.rows() << " float\n";
	for (Eigen::Index i = 0; i < V.rows(); ++i)
	{
		out << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << "\n";
	}

	out << "CELLS " << F.rows() << " " << 4 * F.rows() << "\n";
	for (Eigen::Index i = 0; i < F.rows(); ++i)
	{
		out << "3 " << F(i, 0) << " " << F(i, 1) << " " << F(i, 2) << "\n";
	}

	out << "CELL_TYPES " << F.rows() << "\n";
	for (Eigen::Index i = 0; i < F.rows(); ++i)
	{
		out << "5\n";
	}

	out << "POINT_DATA " << V.rows() << "\n";
	out << "SCALARS mean_curvature float 1\n";
	out << "LOOKUP_TABLE default\n";
	for (Eigen::Index i = 0; i < curvature.mean.rows(); ++i)
	{
		out << curvature.mean(i) << "\n";
	}
	out << "SCALARS gaussian_curvature float 1\n";
	out << "LOOKUP_TABLE default\n";
	for (Eigen::Index i = 0; i < curvature.gaussian.rows(); ++i)
	{
		out << curvature.gaussian(i) << "\n";
	}
	out << "SCALARS principal_curvature1 float 1\n";
	out << "LOOKUP_TABLE default\n";
	for (Eigen::Index i = 0; i < curvature.principal_value1.rows(); ++i)
	{
		out << curvature.principal_value1(i) << "\n";
	}
	out << "SCALARS principal_curvature2 float 1\n";
	out << "LOOKUP_TABLE default\n";
	for (Eigen::Index i = 0; i < curvature.principal_value2.rows(); ++i)
	{
		out << curvature.principal_value2(i) << "\n";
	}

	out << "VECTORS principal_curvature_directoin1 float\n";
	for (Eigen::Index i = 0; i < curvature.principal_directions1.rows(); ++i)
	{
		out << curvature.principal_directions1(i, 0) << " " << curvature.principal_directions1(i, 1) << " " << curvature.principal_directions1(i, 2) << "\n";
	}

	out << "VECTORS principal_curvature_directoin2 float\n";
	for (Eigen::Index i = 0; i < curvature.principal_directions2.rows(); ++i)
	{
		out << curvature.principal_directions2(i, 0) << " " << curvature.principal_directions2(i, 1) << " " << curvature.principal_directions2(i, 2) << "\n";
	}

	out.close();
	return true;
}


bool SaveCsv(
	const std::filesystem::path& filepath,
	const VectorArray& V,
	const std::vector<int>& selected)
{
	std::ofstream out(filepath.c_str());
	if (!out)
	{
		return false;
	}

	for (auto& index : selected)
	{
		out << V(index, 0) << "," << V(index, 1) << "," << V(index, 2) << "\n";
	}

	out.close();
	return true;
}


bool SaveCsv(
	const std::filesystem::path& filepath,
	const VectorArray& V)
{
	std::ofstream out(filepath.c_str());
	if (!out)
	{
		return false;
	}

	for (Eigen::Index i = 0; i < V.rows(); ++i)
	{
		out << V(i, 0) << "," << V(i, 1) << "," << V(i, 2) << "\n";
	}

	out.close();
	return true;
}
