#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <navig_msgs/srv/plan_path.hpp>

#include "path_planner.h"

/* ** *****************************************************************
* Aliases
** ** ****************************************************************/
using NodePtr       = std::shared_ptr<rclcpp::Node>;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using GetMap        = nav_msgs::srv::GetMap;
using GetPlan       = nav_msgs::srv::GetPlan;
using PlanPath      = navig_msgs::srv::PlanPath;
using PathPlanner   = path_planner::PathPlanner;

using GetPlanReqPtr = std::shared_ptr<GetPlan::Request>;
using GetPlanRespPtr= std::shared_ptr<GetPlan::Response>;
using PlnPthReqPtr  = std::shared_ptr<PlanPath::Request>;
using PlnPthRespPtr = std::shared_ptr<PlanPath::Response>;

/* ** *****************************************************************
* Namespace expansion
** ** ****************************************************************/
using namespace std::chrono_literals;

/* ** *****************************************************************
* Glbal variables
** ** ****************************************************************/
rclcpp::Client<GetMap>::SharedPtr cltGetStaticMap       ;
rclcpp::Client<GetMap>::SharedPtr cltGetStaticCostMap   ;
rclcpp::Client<GetMap>::SharedPtr cltGetAugmentedMap    ;
rclcpp::Client<GetMap>::SharedPtr cltGetAugmentedCostMap;
float smooth_alpha   = 0.1;
float smooth_beta    = 0.9;
bool  diagonal_paths = false;
NodePtr node;

/* ** *****************************************************************
* Prototypes
** ** ****************************************************************/
void parse_params();
bool setup_services();

bool callback_a_star_with_static_map(const GetPlanReqPtr req, GetPlanRespPtr resp);
bool callback_a_star_with_augmented_map(const GetPlanReqPtr req, GetPlanRespPtr resp);
bool callback_a_star_local_grid(const PlnPthReqPtr req, PlnPthRespPtr resp);

/* ** *****************************************************************
* Main
** ** ****************************************************************/
int main(int argc, char** argv){
	std::cout << "INITIALIZING PATH CALCULATOR BY MARCO NEGRETE..." << std::endl;
	rclcpp::init(argc, argv);
	node = rclcpp::Node::make_shared("path_planner");
	rclcpp::Rate loop(10);

	parse_params();
	if(!setup_services()) return 1;

	rclcpp::spin(node);
	return 0;
}

/* ** *****************************************************************
* Function definitions
** ** ****************************************************************/

void parse_params(){
	node->declare_parameter("~diagonal_paths", 0.30);
	node->declare_parameter("~smooth_alpha",   0.05);
	node->declare_parameter("~smooth_beta",    1.00);

	diagonal_paths = node->get_parameter("~diagonal_paths").as_double();
	smooth_alpha   = node->get_parameter("~smooth_alpha"  ).as_double();
	smooth_beta    = node->get_parameter("~smooth_beta"   ).as_double();

	std::cout << "PathPlanner.->Calculate diagonal paths: " << (diagonal_paths? "True" : "False") << std::endl;
	std::cout << "PathPlanner.->Smooth alpha: " << smooth_alpha << "   smooth beta: " << smooth_beta << std::endl;
}



bool setup_services(){
	//The following service gets the static map plus the prohibition layer
	cltGetStaticMap        = node->create_client<GetMap>("/map_augmenter/get_static_map"        );
	cltGetStaticCostMap    = node->create_client<GetMap>("/map_augmenter/get_static_cost_map"   );
	cltGetAugmentedMap     = node->create_client<GetMap>("/map_augmenter/get_augmented_map"     );
	cltGetAugmentedCostMap = node->create_client<GetMap>("/map_augmenter/get_augmented_cost_map");

	std::cout << "PathPlanner.->Waiting for map services..." << std::endl;
	while(
		!cltGetStaticMap       ->wait_for_service(1000ms) ||
		!cltGetStaticCostMap   ->wait_for_service(1000ms) ||
		!cltGetAugmentedMap    ->wait_for_service(1000ms) ||
		!cltGetAugmentedCostMap->wait_for_service(1000ms)
	){
		if(!rclcpp::ok){
			RCLCPP_INFO(node->get_logger(), "Interrupted while waiting for the services to be ready. Exiting.");
			return false;
		}
	}
	std::cout << "PathPlanner.->Augmented Map services are now available..." << std::endl;

	node->create_service<GetPlan>("/path_planner/plan_path_with_static",    &callback_a_star_with_static_map);
	node->create_service<GetPlan>("/path_planner/plan_path_with_augmented", &callback_a_star_with_augmented_map);
	node->create_service<PlanPath>("/path_planner/plan_path_local_grid",    &callback_a_star_local_grid);
	return true;
}


bool fetch_static_map(OccupancyGrid& map){
	auto request = std::make_shared<GetMap::Request>();
	auto result = cltGetStaticMap->async_send_request(request);
	if(rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS){
		std::cout << "PathPlanner.-> Cannot get static map!!!!" << std::endl;
		return false;
	}
	map = result.get()->map;
	return true;
}



bool fetch_cost_map(OccupancyGrid& map){
	auto request = std::make_shared<GetMap::Request>();
	auto result = cltGetStaticCostMap->async_send_request(request);
	if(rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS){
		std::cout << "PathPlanner.-> Cannot get static cost map!!!!" << std::endl;
		return false;
	}
	map = result.get()->map;
	return true;
}


bool callback_a_star_with_static_map(const GetPlanReqPtr req, GetPlanRespPtr resp){
	std::cout << "PathPlanner.-> Received path planning request using static map..." << std::endl;
	OccupancyGrid costMap;
	OccupancyGrid staticMap;
	if(!fetch_static_map(staticMap) || !fetch_cost_map(costMap))
		return false;
	bool success = PathPlanner::aStar(staticMap, costMap,
									  req->start.pose, req->goal.pose, diagonal_paths, resp->plan);
	if(success)
		resp->plan = PathPlanner::smoothPath(resp->plan, smooth_alpha, smooth_beta);
	else
		std::cout << "PathPlanner.-> Cannot calculte path from start to goal positions using static map..." << std::endl;
	return success;
}



bool callback_a_star_with_augmented_map(const GetPlanReqPtr req, GetPlanRespPtr resp){
	std::cout << "PathPlanner.-> Received path planning request usign augmented map..." << std::endl;
	OccupancyGrid costMap;
	OccupancyGrid staticMap;
	if(!fetch_static_map(staticMap) || !fetch_cost_map(costMap))
		return false;
	bool success = PathPlanner::aStar(staticMap, costMap,
									  req->start.pose, req->goal.pose, diagonal_paths, resp->plan);
	if(success)
		resp->plan = PathPlanner::smoothPath(resp->plan, smooth_alpha, smooth_beta);
	else
		std::cout << "PathPlanner.-> Cannot calculte path from start to goal positions using augmented map..." << std::endl;
	return success;
}



bool callback_a_star_local_grid(const PlnPthReqPtr req, PlnPthRespPtr resp){
	std::cout << "PathPlanner.-> Received path planning request using local grid map..." << std::endl;
	bool success = PathPlanner::aStar(req->grid_map, req->grid_map, req->start.pose, req->goal.pose, true, resp->plan, req->grid_map.header.frame_id);
	if(success)
		resp->plan = PathPlanner::smoothPath(resp->plan, smooth_alpha, smooth_beta);
	else
		std::cout << "PathPlanner.-> Cannot calculte path from start to goal positions using local grid map..." << std::endl;
	return success;
}

