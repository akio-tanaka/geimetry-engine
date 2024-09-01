#include "curvature_info.h"
#include <igl/cotmatrix.h>
#include <igl/gaussian_curvature.h>
#include <igl/invert_diag.h>
#include <igl/principal_curvature.h>


void Initialize(CurvatureInfo& curvature_info)
{
	curvature_info.mean.resize(0);
	curvature_info.gaussian.resize(0);
	curvature_info.principal_value1.resize(0);
	curvature_info.principal_directions1.resize(0, 3);
	curvature_info.principal_value2.resize(0);
	curvature_info.principal_directions2.resize(0, 3);
}


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


void to_json(nlohmann::json& j, const CurvatureInfo& info)
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