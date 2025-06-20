#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <actionlib_msgs/msg/goal_status.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#define RATE 30

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 11
#define SM_GOAL_POSE_ACCEL 1
#define SM_GOAL_POSE_CRUISE 2
#define SM_GOAL_POSE_DECCEL 3
#define SM_GOAL_POSE_CORRECT_ANGLE 4
#define SM_GOAL_POSE_FINISH 10
#define SM_GOAL_POSE_FAILED 12
#define SM_GOAL_PATH_ACCEL 5
#define SM_GOAL_PATH_CRUISE 6
#define SM_GOAL_PATH_DECCEL 7
#define SM_GOAL_PATH_FINISH 8
#define SM_GOAL_PATH_FAILED 81
#define SM_COLLISION_RISK 9

using Empty            = std_msgs::msg::Empty;
using EmptyPtr         = std::shared_ptr<Empty>;
using Float32          = std_msgs::msg::Float32;
using Float32Ptr       = std::shared_ptr<Float32>;
using Bool             = std_msgs::msg::Bool;
using BoolPtr          = std::shared_ptr<Bool>;
using Path             = nav_msgs::msg::Path;
using PathPtr          = std::shared_ptr<Path>;
using Vector3          = geometry_msgs::msg::Vector3;
using Vector3Ptr       = std::shared_ptr<Vector3>;
using Float32MA        = std_msgs::msg::Float32MultiArray;
using Float32MAPtr     = std::shared_ptr<Float32MA>;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using GoalStatus       = actionlib_msgs::msg::GoalStatus;
using Twist            = geometry_msgs::msg::Twist;
using Float64MA        = std_msgs::msg::Float64MultiArray;

using TfBuffer         = tf2_ros::Buffer;
using TfBufferPtr      = std::unique_ptr<tf2_ros::Buffer>;
using TfListener       = tf2_ros::TransformListener;
using TfListenerPtr    = std::shared_ptr<tf2_ros::TransformListener>;

using NodePtr          = rclcpp::Node::SharedPtr;

float goal_distance  = 0;
float goal_angle     = 0;
bool  new_pose       = false;
bool  new_path       = false;
bool  collision_risk = false;
bool  move_lat       = false;
bool  use_pot_fields = false;
Path goal_path;
Vector3 rejection_force;
bool stop = false;
std::string base_link_name = "base_footprint";

void callback_general_stop(const EmptyPtr msg)
{
    std::cout << "SimpleMove.->General Stop signal received" << std::endl;
    stop     = true;
    new_pose = false;
    new_path = false;
    move_lat = false;
}

void callback_navigation_stop(const EmptyPtr msg)
{
    std::cout << "SimpleMove.->Navigation Stop signal received" << std::endl;
    stop     = true;
    new_pose = false;
    new_path = false;
    move_lat = false;
}

void callback_simple_move_stop(const EmptyPtr msg)
{
    std::cout << "SimpleMove.->Simple move stop signal received" << std::endl;
    stop     = true;
    new_pose = false;
    new_path = false;
}

void callback_goal_dist(const Float32Ptr msg)
{
    std::cout << "SimpleMove.->New move received: goal dist= " << msg->data << std::endl;     
    goal_distance = msg->data;
    goal_angle    = 0;
    new_pose = true;
    new_path = false;
    move_lat = false;
}

void callback_goal_dist_angle(const Float32MAPtr msg)
{
    std::cout << "SimpleMove.->New move received: goal dist= " << msg->data[0] << " and goal angle= " << msg->data[1] << std::endl;
    goal_distance = msg->data[0];
    goal_angle    = msg->data[1];
    new_pose = true;
    new_path = false;
    move_lat = false;
}

void callback_goal_path(const PathPtr msg)
{
    std::cout << "SimpleMove.->New path received with " << msg->poses.size() << " points with id=" << msg->header.frame_id << std::endl;
    move_lat = false;
    if (msg->poses.size() <= 0)
    {
        new_pose = false;
        new_path = false;   
    }else{
        goal_path = *msg;
        new_pose = false;
        new_path = true;
    }
}

void callback_move_lateral(const Float32Ptr msg)
{
    goal_distance = msg->data;
    goal_angle    = 0;
    new_pose = true;
    new_path = false;
    move_lat = true;
}
void callback_collision_risk(const BoolPtr msg)
{
    collision_risk = msg->data;
}

void callback_rejection_force(const Vector3Ptr msg)
{
    rejection_force = *msg;
}

geometry_msgs::msg::Twist calculate_speeds(float robot_x, float robot_y, float robot_t, float goal_x, float goal_y,float min_linear_speed, float max_linear_speed, float angular_speed, float alpha, float beta, bool backwards, bool lateral, bool use_pot_fields=false, double rejection_force_y=0)
{
    float angle_error = 0;
    if(backwards) angle_error = (atan2(robot_y - goal_y, robot_x -goal_x)-robot_t);
    else angle_error = (atan2(goal_y - robot_y, goal_x - robot_x)-robot_t);
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;
    if(lateral) angle_error -= M_PI/2;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;
    if(backwards) max_linear_speed *= -1;
    
    geometry_msgs::msg::Twist result;
    if(lateral)
        result.linear.y   = max_linear_speed  * exp(-(angle_error * angle_error) / (alpha));
    else result.linear.x  = max_linear_speed  * exp(-(angle_error * angle_error) / (alpha));
    result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);

    if(fabs(result.linear.x) < min_linear_speed)
        result.linear.x = 0;
    if(fabs(result.linear.y) < min_linear_speed)
        result.linear.y = 0;
    
    if(use_pot_fields && !lateral) result.linear.y = rejection_force_y;
    return result;
}

geometry_msgs::msg::Twist calculate_speeds(float robot_angle, float goal_angle, float angular_speed, float beta, bool move_lat)
{
    float angle_error = goal_angle - robot_angle;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;

    geometry_msgs::msg::Twist result;
    result.linear.x  = 0;
    result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);
    return result;
}

void get_robot_position(TfBufferPtr& tf_buffer, std::string reference_frame, float& robot_x, float& robot_y, float& robot_t)
{
    TransformStamped t = tf_buffer->lookupTransform(reference_frame, base_link_name, tf2::TimePointZero);
    robot_x = t.transform.translation.x;
    robot_y = t.transform.translation.y;
    tf2::Quaternion q;
    q.setRPY(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);
    // robot_t = atan2(q.z, q.w) * 2;
    robot_t = q.getAngle();
}

void get_goal_position_wrt_odom(float goal_distance, float goal_angle, TfBufferPtr& tf_buffer,
                                float& goal_x, float& goal_y, float& goal_t)
{
    TransformStamped t = tf_buffer->lookupTransform("odom", base_link_name, tf2::TimePointZero);
    float robot_x = t.transform.translation.x;
    float robot_y = t.transform.translation.y;
    tf2::Quaternion q;
    q.setRPY(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);
    // float robot_t = atan2(q.z, q.w) * 2;
    float robot_t = q.getAngle();
    goal_t = robot_t + goal_angle;
    if(goal_t >   M_PI) goal_t -= 2*M_PI;
    if(goal_t <= -M_PI) goal_t += 2*M_PI;
    
    goal_x = robot_x + goal_distance * cos(robot_t + goal_angle + (move_lat?M_PI/2:0));
    goal_y = robot_y + goal_distance * sin(robot_t + goal_angle + (move_lat?M_PI/2:0));
}

float get_path_total_distance(Path& path)
{
    float dist = 0;
    for(size_t i=1; i < path.poses.size(); i++)
        dist += sqrt(pow(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x,2) +
                     pow(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,2));
    return dist;
}

void get_next_goal_from_path(float robot_x, float robot_y, float robot_t, float& goal_x, float& goal_y, int& last_pose_idx, int& next_pose_idx)
{
    if(next_pose_idx >= goal_path.poses.size()) next_pose_idx = goal_path.poses.size() - 1;
    float prev_goal_x = goal_path.poses[last_pose_idx].pose.position.x;
    float prev_goal_y = goal_path.poses[last_pose_idx].pose.position.y;
    float robot_error_x = robot_x - prev_goal_x;
    float robot_error_y = robot_y - prev_goal_y;
    float error = 0;
    do
    {
        goal_x = goal_path.poses[next_pose_idx].pose.position.x;
        goal_y = goal_path.poses[next_pose_idx].pose.position.y;
        float path_vector_x = goal_x - prev_goal_x;
        float path_vector_y = goal_y - prev_goal_y;
        float path_vector_mag = sqrt(path_vector_x*path_vector_x + path_vector_y*path_vector_y);
        path_vector_x = path_vector_mag > 0 ? path_vector_x / path_vector_mag : 0;
        path_vector_y = path_vector_mag > 0 ? path_vector_y / path_vector_mag : 0;
        float scalar_projection = robot_error_x*path_vector_x + robot_error_y*path_vector_y;
        //Error is not euclidean distance, but it is measured projected on the current path segment
        error = path_vector_mag - scalar_projection;
        if(error < 0.25)
            last_pose_idx = next_pose_idx;
    }while(error < 0.25 && ++next_pose_idx < goal_path.poses.size());
}

std_msgs::msg::Float64MultiArray get_next_goal_head_angles(float robot_x, float robot_y, float robot_t, int next_pose_idx)
{
    std_msgs::msg::Float64MultiArray msg;
    int idx = next_pose_idx + 5 >=  goal_path.poses.size() - 1 ? goal_path.poses.size() - 1 : next_pose_idx + 5;
    float goal_x = goal_path.poses[idx].pose.position.x;
    float goal_y = goal_path.poses[idx].pose.position.y;
    float a = atan2(goal_y - robot_y, goal_x - robot_x) - robot_t;
    if(a >   M_PI) a -= 2*M_PI;
    if(a <= -M_PI) a += 2*M_PI;
    msg.data.push_back(a);
    msg.data.push_back(-1.0);
    return msg;
}


void parse_params(NodePtr nodePtr, float& max_linear_speed, float& min_linear_speed,
    float& max_angular_speed, float& alpha, float& beta, float& linear_acceleration,
    float& fine_dist_tolerance, float& coarse_dist_tolerance, float& angle_tolerance,
    bool&  move_head)
{
    nodePtr->declare_parameter("~max_linear_speed",      0.30);
    nodePtr->declare_parameter("~min_linear_speed",      0.05);
    nodePtr->declare_parameter("~max_angular_speed",     1.00);
    nodePtr->declare_parameter("~control_alpha",         0.6548);
    nodePtr->declare_parameter("~control_beta",          0.20);
    nodePtr->declare_parameter("~linear_acceleration",   0.10);
    nodePtr->declare_parameter("~fine_dist_tolerance",   0.03);
    nodePtr->declare_parameter("~coarse_dist_tolerance", 0.20);
    nodePtr->declare_parameter("~angle_tolerance",       0.05);
    nodePtr->declare_parameter("~move_head",             true);
    nodePtr->declare_parameter("~use_pot_fields",        false);
    nodePtr->declare_parameter("/base_link_name", "base_footprint");

    max_linear_speed      = nodePtr->get_parameter("~max_linear_speed"     ).as_double();
    min_linear_speed      = nodePtr->get_parameter("~min_linear_speed"     ).as_double();
    max_angular_speed     = nodePtr->get_parameter("~max_angular_speed"    ).as_double();
    alpha                 = nodePtr->get_parameter("~control_alpha"        ).as_double();
    beta                  = nodePtr->get_parameter("~control_beta"         ).as_double();
    linear_acceleration   = nodePtr->get_parameter("~linear_acceleration"  ).as_double();
    fine_dist_tolerance   = nodePtr->get_parameter("~fine_dist_tolerance"  ).as_double();
    coarse_dist_tolerance = nodePtr->get_parameter("~coarse_dist_tolerance").as_double();
    angle_tolerance       = nodePtr->get_parameter("~angle_tolerance"      ).as_double();
    move_head             = nodePtr->get_parameter("~move_head"            ).as_bool();
    use_pot_fields        = nodePtr->get_parameter("~use_pot_fields"       ).as_bool();
    base_link_name        = nodePtr->get_parameter("/base_link_name"       ).as_string();

    std::cout << "SimpleMove.->Control parameters: min_linear=" << min_linear_speed << " max_linear=" << max_linear_speed;
    std::cout << "  linear_accel=" << linear_acceleration << "  max_angular=" << max_angular_speed << std::endl;
    std::cout << "SimpleMove.->alpha=" << alpha << "  beta=" << beta << "  fine_dist_tol=" << fine_dist_tolerance;
    std::cout << "  coarse_dist_tol=" << coarse_dist_tolerance << "  angle_tol=" << angle_tolerance << std::endl;
    std::cout << "SimpleMove.->Use potential fields rejection force: " << (use_pot_fields ? "True" : "False") << std::endl;
    std::cout << "SimpleMove.->Base link name: " << base_link_name << std::endl;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SIMPLE MOVE NODE BY MARCO NEGRETE..." << std::endl;
    rclcpp::init(argc, argv);
    NodePtr n = std::make_shared<rclcpp::Node>("simple_move");
    
    auto sub_goalDistance     = n->create_subscription<Float32>("/simple_move/goal_dist"      , 1, callback_goal_dist);
    auto sub_goalDistAngle    = n->create_subscription<Float32MA>("/simple_move/goal_dist_angle", 1, callback_goal_dist_angle);
    auto sub_goalPath         = n->create_subscription<Path>("/simple_move/goal_path"      , 1, callback_goal_path);
    auto sub_generalStop      = n->create_subscription<Empty>("/stop", 1, callback_general_stop);
    auto sub_navCtrlStop      = n->create_subscription<Empty>("/navigation/stop",  1, callback_navigation_stop);
    auto sub_navSimpleMvStop  = n->create_subscription<Empty>("/simple_move/stop", 1, callback_simple_move_stop);
    auto sub_gollisionRisk    = n->create_subscription<Bool>("/navigation/obs_detector/collision_risk", 10, callback_collision_risk);
    auto sub_moveLateral      = n->create_subscription<Float32>("/simple_move/goal_dist_lateral" , 1, callback_move_lateral);
    auto sub_rejection_force  = n->create_subscription<Vector3>("/navigation/obs_detector/pf_rejection_force", 1, callback_rejection_force);

    TfBufferPtr tf_buffer = std::make_unique<TfBuffer>(n->get_clock());
    TfListenerPtr tf_listener = std::make_shared<TfListener>( *tf_buffer );
    rclcpp::Rate loop(RATE);

    float max_linear_speed  = 0.3;
    float min_linear_speed  = 0.05;
    float max_angular_speed = 1.0;
    float alpha = 0.6548;
    float beta = 0.2;
    float linear_acceleration = 0.1;
    float fine_dist_tolerance = 0.03;
    float coarse_dist_tolerance = 0.2;
    
    float angle_tolerance = 0.05;
    bool  move_head = true;

    parse_params(n, max_linear_speed, min_linear_speed, max_angular_speed, alpha, beta, linear_acceleration,
        fine_dist_tolerance, coarse_dist_tolerance, angle_tolerance, move_head);


    std::cout << "SimpleMove.->Waiting for odometry and localization transforms..." << std::endl;
    tf_buffer->waitForTransform("map",  base_link_name, rclcpp::Time(0), rclcpp::Duration(1000.0), NULL);
    tf_buffer->waitForTransform("odom", base_link_name, rclcpp::Time(0), rclcpp::Duration(1000.0), NULL);

    auto pub_goal_reached     = n->create_publisher<GoalStatus>("/simple_move/goal_reached", 1);
    auto pub_cmd_vel          = n->create_publisher<Twist>("/cmd_vel", 1);
    auto pub_head_goal_pose   = n->create_publisher<Float64MA>("/hardware/head/goal_pose", 1);

    actionlib_msgs::msg::GoalStatus msg_goal_reached;
    int state = SM_INIT;
    float current_linear_speed = 0;
    float robot_x = 0;
    float robot_y = 0;
    float robot_t = 0;
    float goal_x = 0;
    float goal_y = 0;
    float goal_t = 0;
    float global_goal_x = 0;
    float global_goal_y = 0;
    int prev_pose_idx = 0;
    int next_pose_idx = 0;
    float temp_k = 0;
    int attempts = 0;
    float global_error = 0;
    while(rclcpp::ok())
    {
        if(stop)
        {
            stop = false;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
            pub_cmd_vel->publish(geometry_msgs::msg::Twist());
            pub_goal_reached->publish(msg_goal_reached);
        }
        if(new_pose || new_path)
            state = SM_WAITING_FOR_TASK;
        
        
        switch(state)
        {
        case SM_INIT:
            std::cout << "SimpleMove.->Low level control ready. Waiting for new goal. " << std::endl;
            state = SM_WAITING_FOR_TASK;
            break;

            
        case SM_WAITING_FOR_TASK:
            if(new_pose)
            {
                get_goal_position_wrt_odom(goal_distance, goal_angle, tf_buffer, goal_x, goal_y, goal_t);
                state = SM_GOAL_POSE_ACCEL;
                new_pose = false;
                msg_goal_reached.goal_id.id = "-1";
                attempts = (int)((fabs(goal_distance)*5)/max_linear_speed*RATE + fabs(goal_angle*5)/max_angular_speed*RATE + RATE*5);
            }
            if(new_path)
            {
                state = SM_GOAL_PATH_ACCEL;
                new_path = false;
                prev_pose_idx = 0;
                next_pose_idx = 0;
                global_goal_x = goal_path.poses[goal_path.poses.size() - 1].pose.position.x;
                global_goal_y = goal_path.poses[goal_path.poses.size() - 1].pose.position.y;
                std::stringstream ss;
                // ss << msg->header.frame_id;
                // ss >> msg_goal_reached.goal_id.id;
                attempts = (int)(get_path_total_distance(goal_path)/max_linear_speed*4*RATE + 5*RATE);
            }
            break;

            
        case SM_GOAL_POSE_ACCEL:
            get_robot_position(tf_buffer, "odom", robot_x, robot_y, robot_t);
            global_error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
            if(global_error < fine_dist_tolerance)
                state = SM_GOAL_POSE_CORRECT_ANGLE;
            else if(global_error < current_linear_speed*current_linear_speed/(linear_acceleration*5))
            {
                state = SM_GOAL_POSE_DECCEL;
                temp_k = current_linear_speed/sqrt(global_error);
            }
            else if(current_linear_speed >= max_linear_speed)
            {
                current_linear_speed = max_linear_speed;
                state = SM_GOAL_POSE_CRUISE;
            }
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_ACCEL." << std::endl;
            }
            pub_cmd_vel->publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                 max_angular_speed, alpha*2, beta/4, goal_distance < 0, move_lat));
            current_linear_speed += (linear_acceleration*5)/RATE;
            break;

            
        case SM_GOAL_POSE_CRUISE:
            get_robot_position(tf_buffer, "odom", robot_x, robot_y, robot_t);
            global_error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
            if(global_error < fine_dist_tolerance)
                state = SM_GOAL_POSE_CORRECT_ANGLE;
            else if(global_error < current_linear_speed*current_linear_speed/(linear_acceleration*5))
            {
                temp_k = current_linear_speed/sqrt(global_error);
                state = SM_GOAL_POSE_DECCEL;
            }
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CRUISE." << std::endl;
            }
            pub_cmd_vel->publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                 max_angular_speed, alpha*2, beta/4, goal_distance < 0, move_lat));
            break;

            
        case SM_GOAL_POSE_DECCEL:
            get_robot_position(tf_buffer, "odom", robot_x, robot_y, robot_t);
            global_error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
            if(global_error < fine_dist_tolerance)
                    state = SM_GOAL_POSE_CORRECT_ANGLE;
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_DECCEL." << std::endl;
            }
            current_linear_speed = temp_k * sqrt(global_error);
            if(current_linear_speed < min_linear_speed) current_linear_speed = min_linear_speed;
            pub_cmd_vel->publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                 max_angular_speed, alpha*2, beta/4, goal_distance < 0, move_lat, use_pot_fields, rejection_force.y));
            break;


        case SM_GOAL_POSE_CORRECT_ANGLE:
            get_robot_position(tf_buffer, "odom", robot_x, robot_y, robot_t);
            global_error = (goal_t - robot_t);
            if (global_error > M_PI) global_error-=2*M_PI;
            if (global_error <= -M_PI) global_error+=2*M_PI;
            global_error = fabs(global_error);
            if(global_error < angle_tolerance)
                state = SM_GOAL_POSE_FINISH;
            pub_cmd_vel->publish(calculate_speeds(robot_t, goal_t, max_angular_speed, beta/4, move_lat));
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CORRECT_ANGLE." << std::endl;
            }   
            break;


        case SM_GOAL_POSE_FINISH:
            std::cout << "SimpleMove.->Successful move with dist=" << goal_distance << " angle=" << goal_angle << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
            pub_goal_reached->publish(msg_goal_reached);
            pub_cmd_vel->publish(geometry_msgs::msg::Twist());
            current_linear_speed = 0;
            break;


        case SM_GOAL_POSE_FAILED:
            std::cout << "SimpleMove.->FAILED move with dist=" << goal_distance << " angle=" << goal_angle << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
            pub_goal_reached->publish(msg_goal_reached);
            pub_cmd_vel->publish(geometry_msgs::msg::Twist());
            current_linear_speed = 0;
            break;

                        
        case SM_GOAL_PATH_ACCEL:
            if(collision_risk)
            {
                pub_cmd_vel->publish(geometry_msgs::msg::Twist());
                std::cout << "SimpleMove.->WARNING! Collision risk detected!!!!!" << std::endl;
                state = SM_GOAL_PATH_FAILED;
            }
            else{
                get_robot_position(tf_buffer, goal_path.header.frame_id, robot_x, robot_y, robot_t);
                get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, prev_pose_idx, next_pose_idx);
                global_error = sqrt((global_goal_x - robot_x)*(global_goal_x - robot_x) + (global_goal_y - robot_y)*(global_goal_y - robot_y));
                if(global_error < coarse_dist_tolerance)
                    state = SM_GOAL_PATH_FINISH;
                else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration)
                {
                    state = SM_GOAL_PATH_DECCEL;
                    temp_k = current_linear_speed/sqrt(global_error);
                }
                else if(current_linear_speed >= max_linear_speed)
                {
                    current_linear_speed = max_linear_speed;
                    state = SM_GOAL_PATH_CRUISE;
                }
                if(--attempts <= 0)
                {
                    state = SM_GOAL_PATH_FAILED;
                    std::cout<<"SimpleMove.->Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_ACCEL."<<std::endl;
                }
                pub_cmd_vel->publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, false, move_lat, use_pot_fields, rejection_force.y));
                if(move_head) pub_head_goal_pose->publish(get_next_goal_head_angles(robot_x, robot_y, robot_t, next_pose_idx));
                current_linear_speed += linear_acceleration/RATE;
            }
            break;


        case SM_GOAL_PATH_CRUISE:
            if(collision_risk)
            {
                pub_cmd_vel->publish(geometry_msgs::msg::Twist());
                std::cout << "SimpleMove.->WARNING! Collision risk detected!!!!!" << std::endl;
                state = SM_GOAL_PATH_FAILED;
            }
            else
            {
                get_robot_position(tf_buffer, goal_path.header.frame_id, robot_x, robot_y, robot_t);
                get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, prev_pose_idx, next_pose_idx);
                global_error = sqrt((global_goal_x - robot_x)*(global_goal_x - robot_x) + (global_goal_y - robot_y)*(global_goal_y - robot_y));
                if(global_error < coarse_dist_tolerance)
                    state = SM_GOAL_PATH_FINISH;
                else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration)
                {
                    temp_k = current_linear_speed/sqrt(global_error);
                    state = SM_GOAL_PATH_DECCEL;
                }
                if(--attempts <= 0)
                {
                    state = SM_GOAL_PATH_FAILED;
                    std::cout<<"SimpleMove.->Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_CRUISE."<<std::endl;
                }
                pub_cmd_vel->publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, false,move_lat, use_pot_fields, rejection_force.y));
                if(move_head) pub_head_goal_pose->publish(get_next_goal_head_angles(robot_x, robot_y, robot_t, next_pose_idx));
            }
            break;

            
        case SM_GOAL_PATH_DECCEL:
            if(collision_risk)
            {
                pub_cmd_vel->publish(geometry_msgs::msg::Twist());
                std::cout << "SimpleMove.->WARNING! Collision risk detected!!!!!" << std::endl;
                state = SM_GOAL_PATH_FAILED;
            }
            else
            {
                get_robot_position(tf_buffer, goal_path.header.frame_id, robot_x, robot_y, robot_t);
                get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, prev_pose_idx, next_pose_idx);
                global_error = sqrt((global_goal_x - robot_x)*(global_goal_x - robot_x) + (global_goal_y - robot_y)*(global_goal_y - robot_y));
                if(global_error < coarse_dist_tolerance)
                    state = SM_GOAL_PATH_FINISH;
                if(--attempts <= 0)
                {
                    state = SM_GOAL_PATH_FAILED;
                    std::cout<<"SimpleMove.->Timeout exceeded while trying to reach goal path. Current state:GOAL_PATH_DECCEL."<<std::endl;
                }
                current_linear_speed = temp_k * sqrt(global_error);
                if(current_linear_speed < min_linear_speed) current_linear_speed = min_linear_speed;
                pub_cmd_vel->publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, false,move_lat, use_pot_fields, rejection_force.y));
                if(move_head) pub_head_goal_pose->publish(get_next_goal_head_angles(robot_x, robot_y, robot_t, next_pose_idx));
            }
            break;

            
        case SM_GOAL_PATH_FINISH:
            std::cout << "SimpleMove.->Path succesfully followed." << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
            pub_goal_reached->publish(msg_goal_reached);
            pub_cmd_vel->publish(geometry_msgs::msg::Twist());
            current_linear_speed = 0;
            break;


        case SM_GOAL_PATH_FAILED:
            std::cout << "SimpleMove.->FAILED path traking." << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
            pub_goal_reached->publish(msg_goal_reached);
            pub_cmd_vel->publish(geometry_msgs::msg::Twist());
            current_linear_speed = 0;
            break;

        default:
            std::cout<<"SimpleMove.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE :'(" << std::endl;
            return -1;
        }
        rclcpp::spin_some(n);
        loop.sleep();
    }
}
