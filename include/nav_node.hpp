#ifndef NAV_NODE_HPP
#define NAV_NODE_HPP

// Standard library
#include <vector>
#include "math.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

// Nav_node specific
#include "astar.hpp"

// Messages
#include "cdf_msgs/Pic_Action.h"
#include "cdf_msgs/RobotData.h"
#include "cdf_msgs/MergedDataBis.h"
#include "cdf_msgs/Trajectoire.h"

struct Point
{
    int x;
    int y;
};

class Nav_node
{
    public:
        Nav_node();
        ~Nav_node();

    private:
        // ROS
        ros::NodeHandle n;
        ros::Subscriber sub_robot_data;
        ros::Subscriber sub_merged_data;
        ros::Publisher pub_pic_action;
        ros::Publisher pub_trajectoire;

        // Callbacks
        void robot_data_callback(const cdf_msgs::RobotData::ConstPtr& msg);
        void merged_data_callback(const cdf_msgs::MergedDataBis::ConstPtr& msg);

        // Astar objects
        Astar astar;

        // Robot data
        cdf_msgs::RobotData robot_data;
        Point robot_position;
        Point robot_goal;
        



}




#endif // NAV_NODE_HPP