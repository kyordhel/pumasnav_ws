#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace path_planner{

class PathPlanner
{
public:
	PathPlanner();
	~PathPlanner();

private:
	PathPlanner(const PathPlanner& other) = delete;
	PathPlanner operator=(const PathPlanner& other) = delete;

public:
	static bool aStar(nav_msgs::msg::OccupancyGrid& map,
		nav_msgs::msg::OccupancyGrid& cost_map,
		geometry_msgs::msg::Pose& startPose,
		geometry_msgs::msg::Pose& goalPose,
		bool diagonal_paths,
		nav_msgs::msg::Path& resultPath,
		std::string frame_id="map");

	static nav_msgs::msg::Path smoothPath(
		nav_msgs::msg::Path& path,
		float weight_data = 0.1,
		float weight_smooth = 0.9,
		float tolerance = 0.00001);
};

}
