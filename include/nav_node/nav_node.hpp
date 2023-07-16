#ifndef NAV_NODE_HPP
#define NAV_NODE_HPP

// Standard library
#include <vector>
#include "math.h"
#include "fstream"
#include "rclcpp/rclcpp.hpp"

// Nav_node specific
#include "nav_node/pstar.hpp"

// Messages
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

struct Circle{
    Point center;
    float radius;
};

class Nav_node
{
    public:
        Nav_node();
        ~Nav_node();

        void publish_pic_msg(Point next_goal, bool rayon_courbure);
        void load_map_file(std::string map_file);
        void main_loop_func();

    private:
        // ROS
        ros::NodeHandle n;
        ros::Publisher pub_pic_action;
        ros::Publisher result_pub;
        ros::Subscriber sub_robot_data;
        ros::Subscriber stop_sub;
        ros::Subscriber goal_sub;
        ros::Subscriber obstacles_sub;

        // Variables
        bool standby;
        std::vector<Point> path;
        bool base_map[MAP_WIDTH][MAP_HEIGHT];

        // Callbacks
        void robot_data_callback(const cdf_msgs::RobotData::ConstPtr& msg);
        void stop_callback(const std_msgs::Bool::ConstPtr& msg);
        void goal_callback(const geometry_msgs::Point::ConstPtr& msg);
        void obstacles_callback(const cdf_msgs::MergedDataBis::ConstPtr& msg);

        // Functions
        void get_next_goal();
        void obstacle_processing(Circle obstacle[3]);
        void obstacle_disjunction(cdf_msgs::MergedDataBis obstacles);
        void or_map(bool map[MAP_WIDTH][MAP_HEIGHT], bool map2[MAP_WIDTH][MAP_HEIGHT]);
        void make_circle_map(Circle obstacle, bool map[MAP_WIDTH][MAP_HEIGHT]);

        // Navigation algorithm objects
        PStar nav_alg;

        // Robot data
        cdf_msgs::RobotData robot_data;
        Point robot_position;
        Point robot_goal;
        int robot_number;
        float normal_radius;
        float variation_obs;
        std::string map_file;

        Point next_goal;
        Circle obstacles[3];

        // ROS Parameters
        float goal_tolerance;
};




#endif // NAV_NODE_HPP
