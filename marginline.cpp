#include "marginline.h"
#include <cassert>
#include "curvature_info.h"


void CreateMarginline(
	const VectorArray& V,
	const IndicesArray& F,
	const std::vector<std::vector<int>>& adjacency_list,
	const CurvatureInfo& curvature_info,
	std::vector<int>& marginline,
	std::set<int>& visited)
{
	static const size_t MAX_NUM_TRAVERSAL = 10000;
	static const __int64 NUM_HOPS = 10;

	if (marginline.empty())
	{
		return;
	}

	visited.clear();
	visited.insert(marginline.begin(), marginline.end());

	auto start = marginline.back();
	for (size_t i = 0; i < MAX_NUM_TRAVERSAL; ++i)
	{
		if (marginline.size() > 1)
		{
			if (marginline.front() == marginline.back())
			{
				break;
			}
		}

		auto seed = marginline.back();
		auto neighbors = adjacency_list[seed];

		const Eigen::Vector3d max_curvature_direction = curvature_info.principal_directions1.row(seed);
		const Eigen::Vector3d min_curvature_direction = curvature_info.principal_directions2.row(seed);

		{
			using IndexAndMeanCurvature = std::tuple<int, double>;
			std::vector< IndexAndMeanCurvature> candidates;
			for (size_t j = 0; j < neighbors.size(); ++j)
			{
				auto& neigbor = neighbors[j];
				auto found = visited.find(neigbor);
				if (found != visited.end())
				{
					continue;
				}

				Eigen::Vector3d direction = (V.row(neigbor) - V.row(seed)).normalized();
				assert(marginline.size() > 0);
				auto start = std::max(static_cast<__int64>(0), static_cast<__int64>(marginline.size()) - NUM_HOPS - 1);
				auto end = static_cast<__int64>(marginline.size() - 1);
				auto is_opposite_direction = false;
				for (__int64 k = start; k < end; ++k)
				{
					Eigen::Vector3d existing_direction = (V.row(marginline[k + 1]) - V.row(marginline[k])).normalized();
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
					marginline.push_back(seed);
					visited.insert(neighbors.begin(), neighbors.end());
					continue;
				}
			}
		}

		{
			using IndexAndAbsCos = std::tuple<int, double>;
			std::vector<IndexAndAbsCos> candidates;
			for (size_t j = 0; j < neighbors.size(); ++j)
			{
				auto& neigbor = neighbors[j];
				auto found = visited.find(neigbor);
				if (found != visited.end())
				{
					continue;
				}

				if (curvature_info.mean[seed] > 0 && curvature_info.mean[neigbor] < 0)
				{
					continue;
				}

				Eigen::Vector3d direction = (V.row(neigbor) - V.row(seed)).normalized();
#if 0
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
			marginline.push_back(seed);
			visited.insert(neighbors.begin(), neighbors.end());
		}
	}
}


std::vector<int> DownSampleMarginline(
	const VectorArray& V,
	const IndicesArray& F,
	const std::vector<std::vector<int>>& adjacency_list,
	const CurvatureInfo& curvature_info,
	const std::vector<int>& marginline,
	const std::set<int>& visited,
	size_t num_samples,
	double threshold_to_remove_last_point)
{
	auto linspace = [](int start, int end, int num, bool endpoint) -> std::vector<int>
	{
			std::vector<int> indices;
			auto step = static_cast<double>(end - start) / (endpoint ? num - 1 : num);
			for (auto i = 0; i < num; ++i)
			{
				indices.push_back(static_cast<int>(std::round(start + i * step)));
			}
			return indices;
	};


	auto interval = floor(static_cast<double>(marginline.size()) / num_samples);
	if (interval < 1)
	{
		return marginline;
	}

	auto modulus = marginline.size() % num_samples;
	auto should_remove_last_point = modulus > threshold_to_remove_last_point;
	auto indices = linspace(0, static_cast<int>(marginline.size()) - 1, static_cast<int>(num_samples), should_remove_last_point);
	return indices;

}