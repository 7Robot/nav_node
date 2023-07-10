#include "nav_node.hpp"

Nav_node::Nav_node(){
    //Constructor
    this->pub_pic_action = this->n.advertise<cdf_msgs::Pic_Action>("pic_action", 1000);
    this->sub_robot_data = this->n.subscribe("robot_data", 1000, &Nav_node::robot_data_callback, this);
}

Nav_node::~Nav_node(){
    //Destructor
}

void Nav_node::publish_pic_msg(Point next_goal, bool rayon_courbure)
{
    if (rayon_courbure){
        // Set the curve to 1 mm
        cdf_msgs::Pic_Action curve_msg;
        curve_msg.action_destination = "motor";
        curve_msg.action_msg = "setrayoncourbure 0.001";
        this->pub_pic_action.publish(curve_msg);
    }

    // Set the goal
    cdf_msgs::Pic_Action goal_msg;
    goal_msg.action_destination = "motor";

    // Format the goal
    std::string goal_msg_str = "moveavant "   
        + std::to_string(next_goal.x) 
        + " " + std::to_string(next_goal.y);
    
    goal_msg.action_msg = goal_msg_str;
    this->pub_pic_action.publish(goal_msg);
}

void Nav_node::robot_data_callback(const cdf_msgs::RobotData::ConstPtr& msg)
{
    // Update the robot data
    this->robot_data = *msg;

    // Update the robot position
    this->robot_position.x = (this->robot_data.position).x;
    this->robot_position.y = (this->robot_data.position).y;

    // Activate the navigation
    this->main_loop_func();
}

void Nav_node::main_loop_func(){
    // TODO 
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nav_node");

    Nav_node nav_node;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
    }

}


