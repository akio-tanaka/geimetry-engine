#include "geometry_utils.h"
#include "igl/point_mesh_squared_distance.h"


Eigen::Vector3d Convert(const std::vector<double>& v)
{
	if (v.size() != 3)
	{
		throw std::invalid_argument("the size of the vector must be 3");
	}
	return Eigen::Vector3d(v[0], v[1], v[2]);
}


int FindNearestVertex(const VectorArray& V,	const IndicesArray& F, const Eigen::Vector3d& coordinate)
{
	Eigen::VectorXd sqrD;
	Eigen::VectorXi I;
	Eigen::MatrixXd C;
	igl::point_mesh_squared_distance(coordinate.transpose(), V, F, sqrD, I, C);

	auto nearest_facet = I(0);
	auto nearest_vertex_index = -1;
	auto nearest_distance2 = std::numeric_limits<double>::max();
	for (auto i = 0; i < 3; ++i)
	{
		auto v = F(nearest_facet, i);
		Eigen::RowVector3d vertex = V.row(v);
		auto distance2 = (vertex - coordinate.transpose()).squaredNorm();
		if (distance2 < nearest_distance2)
		{
			nearest_vertex_index = v;
			nearest_distance2 = distance2;
		}
	}

	return nearest_vertex_index;
}