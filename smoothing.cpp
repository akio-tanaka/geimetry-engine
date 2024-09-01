#include "smoothing.h"


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
	// ŠÔˆø‚«
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