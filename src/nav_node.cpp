#include "nav_node.hpp"

float distance(Point a, Point b){
    // Compute the distance between two points
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

bool is_defined(Point a){
    // By default, a point (-1, -1) is undefined
    return (a.x != -1 && a.y != -1);
}

Nav_node::Nav_node(){
    //Constructor
    this->pub_pic_action = this->n.advertise<cdf_msgs::Pic_Action>("pic_action", 1000);
    this->result_pub = this->n.advertise<std_msgs::Bool>("result", 1000);
    this->sub_robot_data = this->n.subscribe("robot_data", 1000, &Nav_node::robot_data_callback, this);
    this->stop_sub = this->n.subscribe("standby", 1000, &Nav_node::stop_callback, this);
    
    this->path = std::vector<Point>();
    
    this->robot_goal.x = -1;
    this->robot_goal.y = -1;
}

Nav_node::~Nav_node(){
    //Destructor
}

void Nav_node::obstacle_processing(Circle obstacle[3]){
    /*
    Given the three obstacles, this function will place circle on the map
    */

    // Obstacle variation ?
    bool variation = false;
    for (int i = 0; i < 3; i++){
        variation = true;
        for (int j = 0; j < 3; j++){
            if (abs(obstacle[i].radius - this->obstacles[j].radius) > 0.01\
                && abs(obstacle[i].center.x - this->obstacles[j].center.x) > 0.01\
                && abs(obstacle[i].center.y - this->obstacles[j].center.y) > 0.01){
                variation = false;
                break;
            }
        }
    }
    if (!variation){
        // If there is no variation, do nothing
        return;
    }
}

void Nav_node::publish_pic_msg(Point next_goal, bool rayon_courbure){
    /*
    This function send position goal to the PIC
    */
    
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

void Nav_node::load_map_file(std::string map_file){
    /*
    Given a file filled with a line of size height*width of 0 and 1, 
    this function will load Nav_node->map with the data
    */
    
    // Get the real path of the file
    std::string real_path = ros::package::getPath("nav_node") + "/maps/" + map_file;
    std::ifstream map_file_stream(real_path);
    
    // Load the line in the map
    std::string line;
    std::getline(map_file_stream, line);
    int length = line.length();
    int x = 0;
    int y = 0;

    bool map[MAP_WIDTH][MAP_HEIGHT];

    for (int i = 0; i < length; i++){
        if (x == MAP_WIDTH){
            x = 0;
            y++;
        }
        if (y == MAP_HEIGHT){
            break;
        }
        if (line[i] == '0'){
            map[x][y] = false;
        }
        else{
            map[x][y] = true;
        }
    }

    // Use accessors to set the map
    this->nav_alg.set_map(map);
}

void Nav_node::robot_data_callback(const cdf_msgs::RobotData::ConstPtr& msg){
    /*
    Get odometry information from the RobotData topic
    */
    
    // Update the robot data
    this->robot_data = *msg;

    // Update the robot position
    this->robot_position.x = (this->robot_data.position).x;
    this->robot_position.y = (this->robot_data.position).y;
}

void Nav_node::stop_callback(const std_msgs::Bool::ConstPtr& msg){
    // Stop everything if a message is published to the standby topic
    this->standby = msg->data;
    if (msg->data){
        // Remove path and goal
        this->path = std::vector<Point>();
        this->robot_goal.x = -1;
        this->robot_goal.y = -1;
    }
}

void Nav_node::goal_callback(const geometry_msgs::Point::ConstPtr& msg){
    /*
    This function will be called when a goal is published to the goal topic
    */
    
    // Update the goal
    this->robot_goal.x = static_cast<int>(msg->x*100);
    this->robot_goal.y = static_cast<int>(msg->y*100);

    // Compute the path
    this->path = this->nav_alg.calculate_path(this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y);
    if (this->path.empty()){
        // If the path is empty, the goal is unreachable
        this->robot_goal.x = -1;
        this->robot_goal.y = -1;
        return;
    }
    else{
        // The path is reachable so we can start the navigation
        this->standby = false;
        this->get_next_goal();
    }
}

void Nav_node::get_next_goal(){
    // Get the next goal
    Point temp = this->path.back();
    this->path.pop_back();

    this->next_goal.x = temp.x;
    this->next_goal.y = temp.y;

    // Publish the goal
    this->publish_pic_msg(this->next_goal, false);
}

void Nav_node::main_loop_func(){
    /*
    This loop function is called every 100 ms, it
    execute the navigation algorithm
    */
    
    if (this->standby){
        // If the robot is in standby, do nothing
        return;
    }

    // If there is no path, compute one
    if (this->path.empty() && is_defined(this->robot_goal)){
        // Compute the path
        this->path = this->nav_alg.calculate_path(this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y);
        this->get_next_goal();
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
        nav_node.main_loop_func();
        ros::spinOnce();
    }
}


