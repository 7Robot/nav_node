#include "nav_node.hpp"

Nav_node::Nav_node(){
    //Constructor
    this->pub_pic_action = this->n.advertise<cdf_msgs::Pic_Action>("pic_action", 1000);
    this->result_pub = this->n.advertise<std_msgs::Bool>("result", 1000);
    this->sub_robot_data = this->n.subscribe("robot_data", 1000, &Nav_node::robot_data_callback, this);
    this->standby_sub = this->n.subscribe("standby", 1000, &Nav_node::standby_callback, this);
    this->path = std::vector<Node>();
}

Nav_node::~Nav_node(){
    //Destructor
}

void Nav_node::publish_pic_msg(Point next_goal, bool rayon_courbure){
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

void Nav_node::robot_data_callback(const cdf_msgs::RobotData::ConstPtr& msg){
    // Update the robot data
    this->robot_data = *msg;

    // Update the robot position
    this->robot_position.x = (this->robot_data.position).x;
    this->robot_position.y = (this->robot_data.position).y;

    // Activate the navigation
    this->main_loop_func();
}

void Nav_node::standby_callback(const std_msgs::Bool::ConstPtr& msg){
    // Update the standby variable
    this->standby = msg->data;
}

void Nav_node::goal_callback(const geometry_msgs::Point::ConstPtr& msg){
    // Update the goal
    this->robot_goal.x = msg->x;
    this->robot_goal.y = msg->y;

    // Compute the path
    this->path = this->nav_alg.calculate_path(this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y);
    if (this->path.empty()){
        // If the path is empty, the goal is unreachable
        this->robot_goal.x = -1;
        this->robot_goal.y = -1;
        return;
    }
    this->get_next_goal();
}

void Nav_node::get_next_goal(){
    // Get the next goal
    this->next_goal = this->path.back();
    this->path.pop_back();

    // Publish the goal
    this->publish_pic_msg(this->next_goal, false);
}

void Nav_node::main_loop_func(){
    if (this->standby){
        // If the robot is in standby, do nothing
        return;
    }

    //TODO: Obstacle processing

    // If there is no path, compute one
    if (this->path.empty() && is_defined(this->robot_goal)){
        // Unexpected error
        ROS_ERROR("ERROR WHILE NAVIGATING: NO PATH ANYMORE");        
    }
    else{
        // If the robot is close enough to the goal, get the next goal
        if (distance(this->robot_position, this->next_goal) < this->goal_tolerance){
            this->get_next_goal();
            if (this->path.empty()){
                // We reached the goal
                this->robot_goal.x = -1;
                this->robot_goal.y = -1;
            }
        }
    }

}

int main(int argc, char * argv[]){
    ros::init(argc, argv, "nav_node");

    Nav_node nav_node;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
    }

}


