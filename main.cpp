#include <iostream>
#include <filesystem>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <igl/adjacency_list.h>
#include <igl/avg_edge_length.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include "type.h"
#include "curvature_info.h"
#include "input.h"
#include "output.h"
#include "io_utils.h"


/////////////////////////////////////////////////////////////////
// Utility functions
/////////////////////////////////////////////////////////////////


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
		std::cout << "Usage: " << argv[0] << " <input json path>\n";
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
