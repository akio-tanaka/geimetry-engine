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
#include "geometry_engine.h"
#include "return_code.h"
#include "io_utils.h"
#include "marginline.h"
#include "smoothing.h"


// #define GUI
GeometryEngine geometry_engine;


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

				std::cout << "clicked vertex index: " << closest_vertex_index << "\n";
				std::cout << "  coordinate: " << V.row(closest_vertex_index) << "\n";

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

	if (!geometry_engine.Initialize(argv[1]))
	{
		std::cout << "failed to initialize the geometry engine\n";
		std::cout << "input.json path: " << argv[1] << "\n";
		return static_cast<int>(geometry_engine.output().return_code);
	}

	auto& V = geometry_engine.V();
	auto& N = geometry_engine.N();
	auto& F = geometry_engine.F();
	auto& adjacency_list = geometry_engine.adjacency_list();
	VectorArray C;

	auto output = geometry_engine.Run();
	if (output.return_code != ToInt(ReturnCode::kSuccess))
	{
		std::cout << "failed to run the geometry engine\n";
		std::cout << output.message << "\n";
		return static_cast<int>(geometry_engine.output().return_code);
	}

	auto& curvature_info = geometry_engine.curvature_info();
	auto minH = curvature_info.mean.minCoeff();
	auto maxH = curvature_info.mean.maxCoeff();

#ifdef GUI
	// curvature direction display
	// Average edge length for sizing
	const double avg = igl::avg_edge_length(V, F);
	const Eigen::RowVector3d red(0.8, 0.2, 0.2), blue(0.2, 0.2, 0.8), white(1.0, 1.0, 1.0);

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

	return 0;
#endif
}
