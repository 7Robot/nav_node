#ifndef NAV_NODE_HPP
#define NAV_NODE_HPP

#ifndef INT_MAX
#define INT_MAX 2147483647
#endif

//#define WORLD_OF_SILENCE 1

// Standard library
#include <vector>
#include "math.h"
#include "fstream"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Nav_node specific
#include "nav_node/pstar.hpp"

// Messages
#include "cdf_msgs/msg/trajectoire.hpp"
#include "cdf_msgs/msg/pic_action.hpp"
#include "cdf_msgs/msg/obstacles.hpp"
#include "cdf_msgs/msg/circle_obstacle.hpp"
#include "cdf_msgs/msg/robot_data.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

class Nav_node : public rclcpp::Node
{
    public:
        Nav_node();
        ~Nav_node();

        void publish_pic_msg(Point next_goal, bool rayon_courbure);
        void load_map_file(std::string map_file);
        void main_loop_func();

    private:
        // ROS
        
        rclcpp::Publisher<cdf_msgs::msg::PicAction>::SharedPtr pub_pic_action;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub;
        rclcpp::Subscription<cdf_msgs::msg::RobotData>::SharedPtr sub_robot_data;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub;
        rclcpp::Subscription<cdf_msgs::msg::Obstacles>::SharedPtr obstacles_sub;
        
        

        // Variables
        bool standby;
        std::vector<Point> path = std::vector<Point>();
        bool base_map[MAP_WIDTH][MAP_HEIGHT];

        // Callbacks
        void robot_data_callback(const cdf_msgs::msg::RobotData msg);
        void stop_callback(const std_msgs::msg::Bool msg);
        void goal_callback(const geometry_msgs::msg::Point msg);
        void obstacles_callback(const cdf_msgs::msg::Obstacles msg);

        // Functions
        void get_next_goal();
        void obstacle_processing(std::vector<cdf_msgs::msg::CircleObstacle> obstacle);
        void or_map(bool map[MAP_WIDTH][MAP_HEIGHT], bool map2[MAP_WIDTH][MAP_HEIGHT]);
        void make_circle_map(cdf_msgs::msg::CircleObstacle obstacle, bool map[MAP_WIDTH][MAP_HEIGHT]);

        // Navigation algorithm objects
        PStar nav_alg;

        // Robot data
        cdf_msgs::msg::RobotData robot_data;
        Point robot_position;
        Point robot_goal;
        int robot_number;
        float normal_radius;
        float variation_obs;
        std::string map_file;

        Point next_goal;
        std::vector<cdf_msgs::msg::CircleObstacle> obstacles;

        // ROS Parameters
        float goal_tolerance;
        rclcpp::TimerBase::SharedPtr timer_;
};




#endif // NAV_NODE_HPP
