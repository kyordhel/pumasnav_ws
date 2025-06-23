#include "path_planner.h"

#include "node.h"
#include <cmath>
#include <queue>
#include <vector>
#include <iostream>


using Point         = geometry_msgs::msg::Point;
using Pose          = geometry_msgs::msg::Pose;
using PoseStamped   = geometry_msgs::msg::PoseStamped;
using Path          = nav_msgs::msg::Path;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

using namespace path_planner;

bool PathPlanner::aStar(OccupancyGrid& map, OccupancyGrid& cost_map, Pose& start_pose,
						Pose& goal_pose, bool diagonal_paths, Path& result_path, std::string frame_id)
{
	std::cout << "PathCalculator.-> Calculating by A* from " << start_pose.position.x << "  ";
	std::cout << start_pose.position.y << "  to " << goal_pose.position.x << "  " << goal_pose.position.y << std::endl;

	int start_x   = (int)((start_pose.position.x - map.info.origin.position.x)/map.info.resolution);
	int start_y   = (int)((start_pose.position.y - map.info.origin.position.y)/map.info.resolution);
	int goal_x    = (int)((goal_pose.position.x  - map.info.origin.position.x)/map.info.resolution);
	int goal_y    = (int)((goal_pose.position.y  - map.info.origin.position.y)/map.info.resolution);
	int idx_start = start_y*map.info.width + start_x;
	int idx_goal  = goal_y *map.info.width + goal_x;
	std::cout << "Start idx=" << idx_start << "  Goal idx=" << idx_goal << std::endl;
	if(map.data[idx_goal] > 20){
		std::cout << "PathPlanner.->Goal point is inside non-free space!!!!" << std::endl;
		return false;
	}
	if(map.data[idx_start] > 20){
		std::cout << "PathPlanner.->Start point is inside non-free space!!!!" << std::endl;
		return false;
	}

	//std::cout << "Points start and end inside valid spaces" << std::endl;
	std::vector<NodePtr> nodes;
	NodePtr current_node;
	int steps = 0;
	nodes.resize(map.data.size());
	int adjacents[8][2] = {{1,0},{0,1},{-1,0},{0,-1},{1,1},{-1,1},{-1,-1},{1,-1}};
	size_t adjacent_n = diagonal_paths ? 8 : 4;


	std::priority_queue<NodePtr, std::vector<NodePtr>, Node::CompareByFValue> open_list;
	for(size_t i=0;  i < map.data.size(); i++)
		nodes[i] = Node::makeShared(i);
	current_node = nodes[idx_start];
	current_node->g_value      = 0;
	current_node->in_open_list = true;
	open_list.push(current_node);

	//std::cout << "Starting main while loop"  << std::endl;
	while(!open_list.empty() && current_node->index != idx_goal){
		current_node = open_list.top();
		open_list.pop();
		current_node->in_closed_list = true;
		//std::cout << "Step: " << steps << std::endl;
		for(size_t i=0; i < adjacent_n; i++){
			//std::cout << "Checking neighbours..." << std::endl;
			int row = current_node->index / map.info.width + adjacents[i][1];
			int col = current_node->index % map.info.width + adjacents[i][0];
			if(row < 0 || row >= map.info.height || col < 0 || col >= map.info.width) //Cells outside map
				continue;
			int n = row*map.info.width + col;
			//std::cout << "Checking node " << n << std::endl;
			if(map.data[n] > 20 || map.data[n] < 0 || nodes[n]->in_closed_list) //Cell in closed list, or occupied or unknown
				continue;
			//std::cout << "Valid neighbour" << std::endl;
			NodePtr neighbor = nodes[n];
			float g_value = current_node->g_value + (i < 4 ? 1.0 : 1.414213562f) + cost_map.data[n];
			float h_value = sqrt((row - goal_y)*(row - goal_y) + (col - goal_x)*(col - goal_x));
			if(g_value < neighbor->g_value){
				neighbor->g_value = g_value;
				neighbor->f_value = g_value + h_value;
				neighbor->parent  = current_node;
			}
			if(!neighbor->in_open_list){
				neighbor->in_open_list = true;
				open_list.push(neighbor);
			}
		}
		steps++;
	}
	std::cout << "PathPlanner.->A* Algorithm ended after " << steps << " steps" << std::endl;
	if(current_node->index != idx_goal)
		return false;

	result_path.header.frame_id = frame_id;
	result_path.poses.clear();
	PoseStamped p;
	p.header.frame_id = frame_id;
	while(current_node->parent != NULL)
	{
		p.pose.position.x = current_node->index % map.info.width * map.info.resolution + map.info.origin.position.x;
		p.pose.position.y = current_node->index / map.info.width * map.info.resolution + map.info.origin.position.y;
		result_path.poses.insert(result_path.poses.begin(), p);
		current_node = current_node->parent;
	}


	std::cout << "PathCalculator.->Resulting path by A* has " << result_path.poses.size() << " points." << std::endl;
	return true;
}

Path PathPlanner::smoothPath(Path& path, float weight_data, float weight_smooth, float tolerance)
{
	Path newPath = path;
	if(path.poses.size() < 3)  return newPath;
	int attempts = 0;
	tolerance*= path.poses.size();
	float change = tolerance + 1;

	while(change >= tolerance && ++attempts < 1000){
		change = 0;
		for(int i=1; i< path.poses.size() - 1; i++){
			Point old_p = path.poses[i].pose.position;
			Point new_p = newPath.poses[i].pose.position;
			Point new_p_next = newPath.poses[i+1].pose.position;
			Point new_p_prev = newPath.poses[i-1].pose.position;
			float last_x = newPath.poses[i].pose.position.x;
			float last_y = newPath.poses[i].pose.position.y;
			new_p.x += weight_data*(old_p.x - new_p.x) + weight_smooth*(new_p_next.x + new_p_prev.x - 2.0*new_p.x);
			new_p.y += weight_data*(old_p.y - new_p.y) + weight_smooth*(new_p_next.y + new_p_prev.y - 2.0*new_p.y);
			change += fabs(new_p.x - last_x) + fabs(new_p.y - last_y);
			newPath.poses[i].pose.position = new_p;
		}
	}
	std::cout << "PathCalculator.->Smoothing finished after " << attempts << " attempts" <<  std::endl;
	return newPath;
}


