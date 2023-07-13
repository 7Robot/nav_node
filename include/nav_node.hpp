#ifndef NAV_NODE_HPP
#define NAV_NODE_HPP

// Standard library
#include <vector>
#include "math.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/console.h"
#include "std_msgs/String.h"

// Nav_node specific
#include "pstar.hpp"

// Messages
#include "cdf_msgs/Pic_Action.h"
#include "cdf_msgs/RobotData.h"
#include "cdf_msgs/MergedDataBis.h"
#include "cdf_msgs/Trajectoire.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"

class Nav_node
{
    public:
        Nav_node();
        ~Nav_node();

        void publish_pic_msg(Point next_goal, bool rayon_courbure);

    private:
        // ROS
        ros::NodeHandle n;
        ros::Publisher pub_pic_action;
        ros::Subscriber sub_robot_data;
        ros::Subscriber standby_sub;
        ros::Subscriber goal_sub;

        // Variables
        bool standby;
        std::vector<Point> path;

        // Callbacks
        void robot_data_callback(const cdf_msgs::RobotData::ConstPtr& msg);
        void standby_callback(const std_msgs::Bool::ConstPtr& msg);
        void goal_callback(const geometry_msgs::Point::ConstPtr& msg);

        // Functions
        void main_loop_func();
        void get_next_goal();

        // Navigation algorithm objects
        PStar nav_alg;

        // Robot data
        cdf_msgs::RobotData robot_data;
        Point robot_position;
        Point robot_goal;

        Point next_goal;

        // ROS Parameters
        float goal_tolerance;
};




#endif // NAV_NODE_HPP
