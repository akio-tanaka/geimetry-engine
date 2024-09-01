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
#include "marginline.h"
#include "smoothing.h"


/////////////////////////////////////////////////////////////////
// Utility functions
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
// hydration
/////////////////////////////////////////////////////////////////

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

				std::vector<int> marginline{ static_cast<int>(closest_vertex_index) };
				std::set<int> visited;
				CreateMarginline(V, F, adjacency_list, curvature_info, marginline, visited);
				if (marginline.size() > 1)
				{
					viewer.data().add_label(V.row(marginline[0]), "0");
					viewer.data().add_points(V.row(marginline[0]), Eigen::RowVector3d(0, 0, 1));
					for (size_t i = 1; i < marginline.size(); ++i)
					{
						std::stringstream ss;
						ss << i;
						viewer.data().add_label(V.row(marginline[i]), ss.str());
						viewer.data().add_points(V.row(marginline[i]), Eigen::RowVector3d(0, 0, 1));
						// viewer.data().add_edges(V.row(loop[i]), V.row(loop[i - 1]), Eigen::RowVector3d(0, 0, 1));
					}
					
					for (auto& vertex_index : marginline)
					{
						visited.erase(vertex_index);
					}
					for (auto& it = visited.begin(); it != visited.end(); ++it)
					{
						viewer.data().add_points(V.row(*it), Eigen::RowVector3d(0, 1, 0));
					}

					SaveCsv("polyline.csv", V, marginline);
					SaveCsv("polyline_smoothed.csv", ChaikinSmoothing(V, marginline, 5));
					SaveCsv("polyline_smoothed2.csv", ChaikinSmoothing2(V, marginline, 5));
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
