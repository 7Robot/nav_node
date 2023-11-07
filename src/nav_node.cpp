#include "nav_node/nav_node.hpp"

using std::placeholders::_1;

float distance(Point a, Point b){
    // Compute the distance between two points
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

bool is_defined(Point a){
    // By default, a point (-1, -1) is undefined
    return (a.x != -1 && a.y != -1);
}

Nav_node::Nav_node() : Node("nav_node"){
    //Constructor    
    this->path = std::vector<Point>();
    
    this->robot_goal.x = -1;
    this->robot_goal.y = -1;
    this->goal_tolerance = 0.1;

    // Get parameters

    this->declare_parameter("robot_id", 0);
    this->declare_parameter("normal_radius", 0.15);
    this->declare_parameter("variation_obs", 0.10);
    this->declare_parameter("map_file", "blank.txt");    

    this->robot_number = this->get_parameter("robot_id").as_int();
    std::string pic_action_topic = "pic_action";
    std::string result_topic = "nav_result";
    std::string robot_data_topic = "robot_data";
    std::string position_goal_topic = "nav_goal";
    std::string stop_topic = "nav_stop";
    std::string obstacle_topic = "raw_obstacles";
    this->normal_radius = this->get_parameter("normal_radius").as_double();
    this->variation_obs = this->get_parameter("variation_obs").as_double();
    this->map_file = this->get_parameter("map_file").as_string();
    

    // Ros Pub and Sub

    this->pub_pic_action = this->create_publisher<cdf_msgs::msg::PicAction>(pic_action_topic, 1000);
    this->result_pub = this->create_publisher<std_msgs::msg::Bool>(result_topic, 1000);
    this->goal_sub = this->create_subscription<geometry_msgs::msg::Point>(position_goal_topic, 1000, std::bind(&Nav_node::goal_callback, this, _1));
    this->sub_robot_data = this->create_subscription<cdf_msgs::msg::RobotData>(robot_data_topic, 1000, std::bind(&Nav_node::robot_data_callback, this, _1));
    this->stop_sub = this->create_subscription<std_msgs::msg::Bool>(stop_topic, 1000, std::bind(&Nav_node::stop_callback, this, _1));
    this->obstacles_sub = this->create_subscription<cdf_msgs::msg::Obstacles>(obstacle_topic, 1000, std::bind(&Nav_node::obstacles_callback, this, _1));

    // Load the map
    this->load_map_file(map_file);

    // Main loop
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Nav_node::main_loop_func, this));

    #ifndef WORlD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "Nav_node initialized");
    #endif
}

Nav_node::~Nav_node(){
    //Destructor
}

void Nav_node::or_map(bool map[MAP_WIDTH][MAP_HEIGHT], bool map2[MAP_WIDTH][MAP_HEIGHT]){
    /*
    This function will do a logical OR between two maps and assign it
    to the first one

	It allows user to add layer containing object like in Gimp
    */
    
    for(int x = 0; x < MAP_WIDTH; x++){
        for(int y = 0; y < MAP_HEIGHT; y++){
            map[x][y] = map[x][y] || map2[x][y];
        }
    }
}

cdf_msgs::msg::CircleObstacle Nav_node::to_absolute_coordinate(cdf_msgs::msg::CircleObstacle circle){
    /* Convert point from the robot coordinate system to 
    the playground coordinate system.

    circle : a circle_obstacle object containing the object coordinate to convert
    robot_position : Position of the robot in the playground coordinate system
    rotation : angle between the x axis of the playground and the front of the robot
    */
    cdf_msgs::msg::CircleObstacle result;

    float sin_p, cos_p;
    sin_p = sin(this->robot_position.theta);
    cos_p = cos(this->robot_position.theta);

    result.center.x = this->robot_position.x +  100 * (cos_p * circle.center.x - sin_p * circle.center.y);
    result.center.y = this->robot_position.y + 100 * (sin_p * circle.center.x - cos_p * circle.center.y);
    result.radius = circle.radius;

    return result;
}


void Nav_node::make_circle_map(cdf_msgs::msg::CircleObstacle obstacle, bool map[MAP_WIDTH][MAP_HEIGHT]){
    /*
    Create a map with a circle obstacle
    */

    for(int x = 0; x < MAP_WIDTH; x++){
        for(int y = 0; y < MAP_HEIGHT; y++){
            if (pow(x - obstacle.center.x, 2) + pow(y - obstacle.center.y, 2) < pow(obstacle.radius, 2)){
                map[x][y] = true;
            }
            else{
                map[x][y] = false;
            }
        }
    }

}

void Nav_node::obstacle_processing(std::vector<cdf_msgs::msg::CircleObstacle> obstacle){
    /*
    Given the three obstacles, this function will place circle on the map
    */

    // Try to see if obstacles are different
    bool variation = true;
    
    int i_len = obstacle.size();
    int j_len = this->obstacles.size();
    if (i_len == j_len){
    for (int i = 0; i < i_len; i++){
        variation = true;
        for (int j = 0; j < j_len; j++){
            if (abs(obstacle[i].center.x - this->obstacles[j].center.x) > this->variation_obs\
                && abs(obstacle[i].center.y - this->obstacles[j].center.y) > this->variation_obs){
                variation = false;
                break;
            }
        }
    }
    }

    if (!variation){
        // If there is no variation, do nothing
        return;
    }
    else {
        #ifndef WORLD_OF_SILENCE
        RCLCPP_INFO(this->get_logger(), "Obstacle variation detected");
        #endif
		
		// Update the map with new obstacles
        bool new_map[MAP_WIDTH][MAP_HEIGHT];
        for (int x = 0; x < MAP_WIDTH; x++){
            for (int y = 0; y < MAP_HEIGHT; y++){
                new_map[x][y] = this->base_map[x][y];
            }
        }
        bool temp_map[MAP_WIDTH][MAP_HEIGHT];
        // If there is a variation, update the obstacles
        this->obstacles.clear();
        for (int i = 0; i < i_len; i++){
            RCLCPP_INFO(this->get_logger(), "Obstacle : (%f,%f)", obstacle.at(i).center.x, obstacle.at(i).center.y);
            this->obstacles.push_back(obstacle.at(i));
            this->make_circle_map(obstacle.at(i), temp_map);
            this->or_map(new_map, temp_map);            
        }
    }
}

void Nav_node::publish_pic_msg(Point next_goal, bool rayon_courbure){
    /*
    This function send position goal to the PIC
    */

    if (rayon_courbure){
        // Set the curve to 1 mm
        cdf_msgs::msg::PicAction curve_msg;
        curve_msg.action_destination = "motor";
        curve_msg.action_msg = "setrayoncourbure 0.001";
        this->pub_pic_action->publish(curve_msg);
    }

    // Set the goal
    cdf_msgs::msg::PicAction goal_msg;
    goal_msg.action_destination = "motor";

    // Format the goal

    std::string goal_msg_str = "moveavant "   
        + std::to_string(next_goal.x/100.0) 
        + " " + std::to_string(next_goal.y/100.0);
    
    goal_msg.action_msg = goal_msg_str;
    this->pub_pic_action->publish(goal_msg);
    #ifndef WORLD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "Goal sent to PIC : %s", goal_msg_str.c_str());
    #endif
}

void Nav_node::load_map_file(std::string map_file){
    /*
    Given a file filled with a line of size height*width of 0 and 1, 
    this function will load Nav_node->map with the data
    */
    
    // Get the real path of the file
    std::string real_path = ament_index_cpp::get_package_share_directory("nav_node") + "/../../../../src/nav_node/maps/" + map_file;
    std::cout << real_path << std::endl;

    std::ifstream map_file_stream(real_path);
    
    // Load the line in the map
    std::string line;
    std::getline(map_file_stream, line);
    int length = line.length();
    if (length != MAP_WIDTH * MAP_HEIGHT){
        #ifndef WORLD_OF_SILENCE
        RCLCPP_ERROR(this->get_logger(), "Map file %s is not valid, size : %d", map_file.c_str(), length);
        #endif
    }
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
        x++;
    }

    // Use accessors to set the map
    for (int x = 0; x < MAP_WIDTH; x++){
        for (int y = 0; y < MAP_HEIGHT; y++){
            this->base_map[x][y] = map[x][y];
        }
    }
    this->nav_alg.set_map(map);

    #ifndef WORLD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "Map %s loaded", map_file.c_str());
    #endif
}

void Nav_node::obstacles_callback(const cdf_msgs::msg::Obstacles msg){
    /*
    This function gather the msg from the cluster detector and process all the 
    obstacles to add them to the map.
    */
    std::vector<cdf_msgs::msg::CircleObstacle> obstacle;
    cdf_msgs::msg::CircleObstacle tmp_circle;
    
    int range_msg = msg.circles.size();
    for (int i = 0; i < range_msg; i++){
        tmp_circle = this->to_absolute_coordinate(msg.circles[i]);

        // If the obstacle is in the playground we can add it
        if (tmp_circle.center.x > 0 && 
        tmp_circle.center.x < MAP_WIDTH &&    
        tmp_circle.center.y > 0 &&    
        tmp_circle.center.y < MAP_HEIGHT){
            obstacle.push_back(tmp_circle);
        }   
    }

    this->obstacle_processing(obstacle);
}


void Nav_node::robot_data_callback(const cdf_msgs::msg::RobotData msg){
    /*
    Get odometry information from the RobotData topic
    */
    
    // Update the robot data
    this->robot_data = msg;    

    // Update the robot position
    this->robot_position.x = (this->robot_data.position).x * 100;
    this->robot_position.y = (this->robot_data.position).y * 100;
    this->robot_position.theta = (this->robot_data.position).z;

    #ifndef WORLD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "Robot position : (%d, %d)", this->robot_position.x, this->robot_position.y);
    #endif
}

void Nav_node::stop_callback(const std_msgs::msg::Bool msg){
    // Stop everything if a message is published to the standby topic
    this->standby = msg.data;
    if (msg.data){
        // Remove path and goal
        this->path = std::vector<Point>();
        this->robot_goal.x = -1;
        this->robot_goal.y = -1;
    }
    #ifndef WORLD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "Standby : %d", this->standby);
    #endif
}

void Nav_node::goal_callback(const geometry_msgs::msg::Point msg){
    /*
    This function will be called when a goal is published to the goal topic
    */
    
    // Update the goal
    this->robot_goal.x = static_cast<int>(msg.x*100);
    this->robot_goal.y = static_cast<int>(msg.y*100);

    // Compute the path
    
    int result = this->nav_alg.calculate_path(this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y, this->path);
    if (result != 0){
        // If the result is different from 0 there is an error, please see errno description in README.md
        
        #ifndef WORLD_OF_SILENCE
        if (result == -1){
            RCLCPP_WARN(this->get_logger(), "Path from (%d, %d) to (%d, %d) is unreachable", this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y);
        }
        else if (result == -2){
            RCLCPP_WARN(this->get_logger(), "Start (%d, %d) is in obstacle", this->robot_position.x, this->robot_position.y);
        }
        else if (result == -3){
            RCLCPP_WARN(this->get_logger(), "End (%d, %d) is in obstacle", this->robot_goal.x, this->robot_goal.y);
        }
        #endif
        this->robot_goal.x = -1;
        this->robot_goal.y = -1;
        return;
    }
    else{
        // The path is reachable so we can start the navigation
        #ifndef WORLD_OF_SILENCE
        RCLCPP_INFO(this->get_logger(), "Path from (%d, %d) to (%d, %d) is reachable :", this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y);
        int tmp = this->path.size();
        for (int i = 0; i < tmp; i++){
            RCLCPP_INFO(this->get_logger(), "(%d, %d)", this->path[i].x, this->path[i].y);
        }
        #endif
        this->standby = false;
        this->get_next_goal();
    }

    #ifndef WORLD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "New goal : (%d, %d)", this->robot_goal.x, this->robot_goal.y);
    #endif
}

void Nav_node::get_next_goal(){
    // Get the next goal
    Point temp = this->path.back();
    this->path.pop_back();

    this->next_goal.x = temp.x;
    this->next_goal.y = temp.y;

    #ifndef WORLD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "Next goal : (%d, %d)", this->next_goal.x, this->next_goal.y);
    #endif
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

    // If the robot is close enough to the goal, get the next goal
    if (!(this->path.empty()) && distance(this->robot_position, this->next_goal) < this->goal_tolerance){
        this->get_next_goal();
        if (this->path.empty()){
            // We reached the goal
            #ifndef WORLD_OF_SILENCE
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            #endif
            this->robot_goal.x = -1;
            this->robot_goal.y = -1;
        }
    }
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav_node>());
    rclcpp::shutdown();
    return 0;
}
