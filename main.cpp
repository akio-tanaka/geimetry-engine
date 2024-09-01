#include <algorithm>
#include <cassert>
#include <iostream>
#include <filesystem>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <igl/adjacency_list.h>
#include <igl/avg_edge_length.h>
#include <igl/cotmatrix.h>
#include <igl/gaussian_curvature.h>
#include <igl/invert_diag.h>
#include <igl/massmatrix.h>
#include <igl/parula.h>
#include <igl/per_corner_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/principal_curvature.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/readPLY.h>
#include <igl/readSTL.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include <nlohmann/json.hpp>


using json = nlohmann::json;
using ScalarArray = Eigen::VectorXd;
using VectorArray = Eigen::MatrixXd;
using IndicesArray = Eigen::MatrixXi;


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
 * @brief Convert CurvatureInfo to json
 * @param j json object
 * @param info curvature information
 * @return void
 */
void to_json(json& j, const CurvatureInfo& info)
{
	auto convert = [](const VectorArray& vec) -> std::vector<std::vector<double>>
	{
		std::vector<std::vector<double>> result(vec.rows());
		for (Eigen::Index i = 0; i < vec.rows(); i++)
		{
			result[i].resize(3);
			for (Eigen::Index j = 0; j < 3; j++)
			{
				result[i][j] = vec(i, j);
			}
		}
		return result;
	};

	j["mean"] = info.mean;
	j["gaussian"] = info.gaussian;
	j["principal_value1"] = info.principal_value1;
	j["principal_value2"] = info.principal_value2;
	j["principal_directions1"] = convert(info.principal_directions1);
	j["principal_directions2"] = convert(info.principal_directions2);
}

/////////////////////////////////////////////////////////////////
// Utility functions
/////////////////////////////////////////////////////////////////

/**
 * @brief Load a model file
 * @param path path to the model file
 * @param V vertices
 * @param F faces
 * @return true if the model is loaded successfully, false otherwise
 */
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


/**
 * @brief Calculate curvatures
 * @param V vertices
 * @param F faces
 * @param curvature_info curvature information
 * @return void
 */
void CalcCurvatures(const VectorArray& V, const IndicesArray& F, CurvatureInfo& curvature_info)
{
	// Alternative discrete mean curvature
	VectorArray HN;
	Eigen::SparseMatrix<VectorArray::value_type> L, M, Minv;
	igl::cotmatrix(V, F, L);
	igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_VORONOI, M);
	igl::invert_diag(M, Minv);

	// Laplace-Beltrami of position
	HN = -Minv * (L * V);

	// Extract magnitude as mean curvature
	// Compute curvature directions via quadric fitting
	igl::principal_curvature(V, F,
		curvature_info.principal_directions1,
		curvature_info.principal_directions2,
		curvature_info.principal_value1,
		curvature_info.principal_value2);
	curvature_info.mean = HN.rowwise().norm();
	curvature_info.mean = static_cast<VectorArray::value_type>(0.5) * (curvature_info.principal_value1 + curvature_info.principal_value2);
	igl::gaussian_curvature(V, F, curvature_info.gaussian);
}


/**
 * @brief Save curvatures to a json file
 * @param filepath file path
 * @param info curvature information
 * @return true if the curvatures are saved successfully, false otherwise
 */
bool SaveCurvatures(const std::filesystem::path& filepath, const CurvatureInfo& info)
{
	try
	{
		std::ofstream out(filepath.c_str());
		json json_obj;
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


/**
 * @brief Save a polyline to a file
 * @param filepath file path
 * @param V vertices
 * @param selected selected vertices
 * @return true if the polyline is saved successfully, false otherwise
 */
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


/////////////////////////////////////////////////////////////////
//smoothing
/////////////////////////////////////////////////////////////////

/**
* @brief Chaikin smoothing
* @param V vertices
* @param loop loop
* @param num_iterations number of iterations
* @return smoothed vertices
*/
VectorArray ChaikinSmoothing(const VectorArray& V, const std::vector<int>& loop, int num_iterations)
{
	VectorArray smoothed(loop.size(), 3);
	for (size_t i = 0; i < loop.size(); ++i)
	{
		smoothed.row(i) = V.row(loop[i]);
	}

	for (auto i = 0; i < num_iterations; ++i)
	{
		std::vector<Eigen::Vector3d> new_loop;
		for (auto j = 0; j < smoothed.rows() - 1; ++j)
		{
			Eigen::Vector3d p0 = smoothed.row(j);
			Eigen::Vector2d p0_yz = p0.tail<2>();
			Eigen::Vector3d p1 = smoothed.row(j + 1);
			Eigen::Vector2d p1_yz = p1.tail<2>();

			Eigen::Vector2d q = 0.75 * p0_yz + 0.25 * p1_yz;
			Eigen::Vector2d r = 0.25 * p0_yz + 0.75 * p1_yz;

			new_loop.push_back(Eigen::Vector3d(0, q(0), q(1)));
			new_loop.push_back(Eigen::Vector3d(0, r(0), r(1)));
		}
		new_loop.push_back(smoothed.row(smoothed.rows() - 1));

		smoothed.resize(new_loop.size(), 3);
		for (size_t j = 0; j < new_loop.size(); ++j)
		{
			smoothed.row(j) = new_loop[j];
		}
	}
	return smoothed;
}


VectorArray ChaikinSmoothing2(const VectorArray& V, const std::vector<int>& loop, int num_iterations)
{
	// 間引き
	if (loop.size() < 3)
	{
		return {};
	}

	size_t num_points = 30;
	size_t interval = static_cast<size_t>((loop.size() - 2) / (num_points - 2)) + 1;
	std::vector<size_t> target_indices;
	for (size_t i = 0; i < loop.size(); i += interval)
	{
		target_indices.push_back(i);
	}
	if (target_indices.back() != loop.size() - 1)
	{
		target_indices.push_back(loop.size() - 1);
	}
	size_t xidata_size = target_indices.size();


	VectorArray smoothed(target_indices.size(), 3);
	for (size_t i = 0; i < target_indices.size(); ++i)
	{
		smoothed.row(i) = V.row(target_indices[i]);
	}

	for (auto i = 0; i < num_iterations; ++i)
	{
		std::vector<Eigen::Vector3d> new_loop;
		for (auto j = 0; j < smoothed.rows() - 1; ++j)
		{
			Eigen::Vector3d p0 = smoothed.row(j);
			Eigen::Vector2d p0_yz = p0.tail<2>();
			Eigen::Vector3d p1 = smoothed.row(j + 1);
			Eigen::Vector2d p1_yz = p1.tail<2>();

			Eigen::Vector2d q = 0.75 * p0_yz + 0.25 * p1_yz;
			Eigen::Vector2d r = 0.25 * p0_yz + 0.75 * p1_yz;

			new_loop.push_back(Eigen::Vector3d(0, q(0), q(1)));
			new_loop.push_back(Eigen::Vector3d(0, r(0), r(1)));
		}
		new_loop.push_back(smoothed.row(smoothed.rows() - 1));

		smoothed.resize(new_loop.size(), 3);
		for (size_t j = 0; j < new_loop.size(); ++j)
		{
			smoothed.row(j) = new_loop[j];
		}
	}
	return smoothed;
}


/////////////////////////////////////////////////////////////////
// hydration
/////////////////////////////////////////////////////////////////
void Traverse(
	const VectorArray& V,
	const IndicesArray& F,
	const std::vector<std::vector<int>>& adjacency_list,
	const CurvatureInfo& curvature_info,
	std::vector<int>& result,
	std::set<int>& visited)
{
	static const size_t MAX_NUM_TRAVERSAL = 10000;
	static const __int64 NUM_HOPS = 10;

	if (result.empty())
	{
		return;
	}

	visited.clear();
	visited.insert(result.begin(), result.end());

	auto start = result.back();
	for (size_t i = 0; i < MAX_NUM_TRAVERSAL; ++i)
	{
		if (result.size() > 1)
		{
			if (result.front() == result.back())
			{
				break;
			}
		}

		auto seed = result.back();
		auto neighbors = adjacency_list[seed];
		
		const Eigen::Vector3d max_curvature_direction = curvature_info.principal_directions1.row(seed);
		const Eigen::Vector3d min_curvature_direction = curvature_info.principal_directions2.row(seed);

		// ���͂ɖK�⒆�̒��_�����傫�����ϋȗ������ߖT���_������ꍇ�͂�����Ɉړ�
		{
			using IndexAndMeanCurvature = std::tuple<int, double>;
			std::vector< IndexAndMeanCurvature> candidates;
			for (size_t j = 0; j < neighbors.size(); ++j)
			{
				auto& neigbor = neighbors[j];

				// �K��ς̒��_�̓X�L�b�v
				auto found = visited.find(neigbor);
				if (found != visited.end())
				{
					continue;
				}

				// �t�����ɐi�ޏꍇ�̓X�L�b�v
				Eigen::Vector3d direction = (V.row(neigbor) - V.row(seed)).normalized();
				assert(result.size() > 0);
				auto start = std::max(static_cast<__int64>(0), static_cast<__int64>(result.size()) - NUM_HOPS - 1);
				auto end = static_cast<__int64>(result.size() - 1);
				auto is_opposite_direction = false;
				for (__int64 k = start; k < end; ++k)
				{
					Eigen::Vector3d existing_direction = (V.row(result[k + 1]) - V.row(result[k])).normalized();
					if (direction.dot(existing_direction) < 0.0)
					{
						is_opposite_direction = true;
						break;
					}
				}
				if (is_opposite_direction)
				{
					continue;
				}

				candidates.push_back(std::make_tuple(neigbor, curvature_info.mean[neigbor]));
			}
			if (!candidates.empty())
			{
				auto max_element = std::max_element(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs)
					{
						return std::get<1>(lhs) < std::get<1>(rhs);
					});
				if (std::get<1>(*max_element) > curvature_info.mean[seed])
				{
					seed = std::get<0>(*max_element);
					result.push_back(seed);
					visited.insert(neighbors.begin(), neighbors.end());
					continue;
				}
			}
		}

		// �K�⒆�̒��_�̕��ϋȗ����ߖT�����傫���ꍇ�A���ŏ��ȗ������ɍł��߂����_�Ɉړ�
		{
			using IndexAndAbsCos = std::tuple<int, double>;
			std::vector<IndexAndAbsCos> candidates;
			for (size_t j = 0; j < neighbors.size(); ++j)
			{
				auto& neigbor = neighbors[j];

				// �K��ς̒��_�̓X�L�b�v
				auto found = visited.find(neigbor);
				if (found != visited.end())
				{
					continue;
				}

				// ���ϋȗ��������畉�ɕω�����悤�Ȉړ��̓X�L�b�v
				if (curvature_info.mean[seed] > 0 && curvature_info.mean[neigbor] < 0)
				{
					continue;
				}

				Eigen::Vector3d direction = (V.row(neigbor) - V.row(seed)).normalized();
#if 0
				// �t�����ɐi�ޏꍇ�̓X�L�b�v
				assert(result.size() > 0);
				auto start = std::max(static_cast<__int64>(0), static_cast<__int64>(result.size()) - NUM_HOPS - 1);
				auto end = static_cast<__int64>(result.size() - 1);
				auto is_opposite_direction = false;
				for (__int64 k = start; k < end; ++k)
				{
					Eigen::Vector3d existing_direction = (V.row(result[k + 1]) - V.row(result[k])).normalized();
					if (direction.dot(existing_direction) < 0.0)
					{
						is_opposite_direction = true;
						break;
					}
				}
				if (is_opposite_direction)
				{
					continue;
				}
#endif

				candidates.push_back(std::make_tuple(neigbor, std::abs(direction.dot(min_curvature_direction))));
			}
			if (candidates.empty())
			{
				break;
			}

			auto next = std::max_element(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs)
				{
					return std::get<1>(lhs) < std::get<1>(rhs);
				});
			seed = std::get<0>(*next);
			result.push_back(seed);
			visited.insert(neighbors.begin(), neighbors.end());
		}
	}
}


void HydrateSelectionWithCurvature(
	igl::opengl::glfw::Viewer& viewer,
	VectorArray& V,
	IndicesArray& F,
	std::vector<std::vector<int>>& adjacency_list,
	const CurvatureInfo& curvature_info)
{
	// how to change thickness of edges as overlays: https://github.com/libigl/libigl/issues/1270
	static const Eigen::RowVector3d SELECTED_EDGE_COLOR(0, 0, 0);
	static const Eigen::RowVector3d SELECTED_VERTEX_COLOR(1, 0, 0);
	static const Eigen::RowVector3d CURVATURE_EDGE_COLOR(1, 1, 0);

	viewer.callback_mouse_up = [&V, &F, &adjacency_list, &curvature_info](igl::opengl::glfw::Viewer& viewer, int, int) -> bool
		{
			int fid = 0;
			Eigen::Vector3d bc;
			auto x = static_cast<float>(viewer.current_mouse_x);
			auto y = static_cast<float>(viewer.core().viewport(3) - viewer.current_mouse_y);
			if (igl::unproject_onto_mesh(
				Eigen::Vector2f(x, y),
				viewer.core().view,
				viewer.core().proj,
				viewer.core().viewport,
				V,
				F,
				fid,
				bc))
			{
				// showing clicked face
				viewer.data().add_edges(V.row(F(fid, 0)), V.row(F(fid, 1)), SELECTED_EDGE_COLOR);
				viewer.data().add_edges(V.row(F(fid, 1)), V.row(F(fid, 2)), SELECTED_EDGE_COLOR);
				viewer.data().add_edges(V.row(F(fid, 2)), V.row(F(fid, 0)), SELECTED_EDGE_COLOR);

				// get closest vertex from clicking point
				const auto bc_arr = std::vector<double>{ bc[0], bc[1], bc[2] };
				const auto closest_index = std::max_element(bc_arr.begin(), bc_arr.end()) - bc_arr.begin();
				const auto closest_vertex_index = F(fid, closest_index);
				viewer.data().add_points(V.row(closest_vertex_index), SELECTED_VERTEX_COLOR);

				//auto mean_curvature = curvature_info.mean[closest_vertex_index];
				//const Eigen::Vector3d max_curvature_direction = curvature_info.principal_directions1.row(closest_vertex_index);
				//const Eigen::Vector3d min_curvature_direction = curvature_info.principal_directions2.row(closest_vertex_index);

				//auto& neighbors = adjacency_list[closest_vertex_index];
				//auto next_index = std::max_element(neighbors.begin(), neighbors.end(), [&V, &closest_vertex_index, &min_curvature_direction](const auto& lhs, const auto& rhs)
				//	{
				//		Eigen::Vector3d lhs_direction = (V.row(lhs) - V.row(closest_vertex_index)).normalized();
				//		Eigen::Vector3d rhs_direction = (V.row(rhs) - V.row(closest_vertex_index)).normalized();
				//		return std::abs(lhs_direction.dot(min_curvature_direction)) < std::abs(rhs_direction.dot(min_curvature_direction));
				//	}) - neighbors.begin();
				//viewer.data().add_points(V.row(neighbors[next_index]), Eigen::RowVector3d(0, 0, 1));
				//viewer.data().add_edges(V.row(closest_vertex_index), V.row(neighbors[next_index]), CURVATURE_EDGE_COLOR);

				std::vector<int> loop{ static_cast<int>(closest_vertex_index) };
				std::set<int> visited;
				Traverse(V, F, adjacency_list, curvature_info, loop, visited);
				if (loop.size() > 1)
				{
					viewer.data().add_label(V.row(loop[0]), "0");
					viewer.data().add_points(V.row(loop[0]), Eigen::RowVector3d(0, 0, 1));
					for (size_t i = 1; i < loop.size(); ++i)
					{
						std::stringstream ss;
						ss << i;
						viewer.data().add_label(V.row(loop[i]), ss.str());
						viewer.data().add_points(V.row(loop[i]), Eigen::RowVector3d(0, 0, 1));
						// viewer.data().add_edges(V.row(loop[i]), V.row(loop[i - 1]), Eigen::RowVector3d(0, 0, 1));
					}
					
					for (auto& vertex_index : loop)
					{
						visited.erase(vertex_index);
					}
					for (auto& it = visited.begin(); it != visited.end(); ++it)
					{
						viewer.data().add_points(V.row(*it), Eigen::RowVector3d(0, 1, 0));
					}

					SaveCsv("polyline.csv", V, loop);
					SaveCsv("polyline_smoothed.csv", ChaikinSmoothing(V, loop, 5));
					SaveCsv("polyline_smoothed2.csv", ChaikinSmoothing2(V, loop, 5));
				}

				return true;
			}
			return false;
		};

	viewer.callback_key_up = [&V, &F, &adjacency_list, &curvature_info](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) -> bool
		{
			if (key == 'r' || key == 'R')
			{
				viewer.data().clear_points();
				viewer.data().clear_labels();
				// viewer.data().clear_edges();
			}
			return false;
		};
}



/////////////////////////////////////////////////////////////////
// main function
/////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		std::cout << "Usage: " << argv[0] << " <model file path>\n";
		return 1;
	}

	VectorArray V, N, C;
	IndicesArray F;
	std::filesystem::path filepath = argv[1];
	if (!LoadModel(filepath, V, F))
	{
		return 2;
	}
	std::vector<std::vector<int> > adjacency_list;
	igl::adjacency_list(F, adjacency_list);
	std::cout << "done to load: " << filepath << "\n";

	CurvatureInfo curvature_info;
	CalcCurvatures(V, F, curvature_info);
	auto minH = curvature_info.mean.minCoeff();
	auto maxH = curvature_info.mean.maxCoeff();

	std::cout << "minH = " << minH << "\n";
	std::cout << "maxH = " << maxH << "\n";

	auto json_filepath = filepath.parent_path() / "curvatures.json";
	if (!SaveCurvatures(json_filepath, curvature_info))
	{
		std::cout << "failed to save curvatures\n";
		return 3;
	}

	auto vtk_filepath = std::filesystem::path(filepath).replace_extension(".vtk");
	if (!SaveVtk(vtk_filepath, V, F, curvature_info))
	{
		std::cout << "failed to save vtk file\n";
		return 4;
	}

	// curvature direction display
	// Average edge length for sizing
	const double avg = igl::avg_edge_length(V, F);
	const Eigen::RowVector3d red(0.8, 0.2, 0.2), blue(0.2, 0.2, 0.8), white(1.0, 1.0, 1.0);


#if 1
	// Plot the mesh
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
	viewer.data().set_data(curvature_info.mean, -0.1, 0.1, igl::COLOR_MAP_TYPE_JET);
	// viewer.data().set_data(info.mean);
	viewer.data().set_face_based(true);
	viewer.data().show_lines = false;

	// viewer.data().add_edges(V + info.principal_directions1 * avg, V - info.principal_directions1 * avg, red);
	viewer.data().add_edges(V + curvature_info.principal_directions2 * avg, V - curvature_info.principal_directions2 * avg, white);

	// hydration
	HydrateSelectionWithCurvature(viewer, V, F, adjacency_list, curvature_info);

	// give white color to the background
	// viewer.core().background_color.setOnes();
	
	viewer.launch();
#endif
	return 0;
}
