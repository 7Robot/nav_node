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

    // Get parameters
    std::string pic_action_topic = "/pic_action";
    std::string result_topic = "/result";
    std::string robot_data_topic = "/robot_data";
    std::string stop_topic = "/stop";
    std::string position_goal_topic = "/position_goal";
    std::string map_file = "blank.txt";
    this->goal_tolerance = 10;

    /*
    this->n.getParam("robot_id", this->robot_number);
    this->n.getParam("pic_action_topic", pic_action_topic);
    this->n.getParam("result_topic", result_topic);
    this->n.getParam("robot_data_topic", robot_data_topic);
    this->n.getParam("position_goal_topic", position_goal_topic);
    this->n.getParam("stop_topic", stop_topic);
    this->n.getParam("normal_radius", this->normal_radius);
    this->n.getParam("variation_obs", this->variation_obs);
    this->n.getParam("map_file", this->map_file);
    */

    // Ros Pub and Sub

    this->pub_pic_action = this->create_publisher<cdf_msgs::msg::PicAction>(pic_action_topic, 1000);
    this->result_pub = this->create_publisher<std_msgs::msg::Bool>(result_topic, 1000);
    this->goal_sub = this->create_subscription<geometry_msgs::msg::Point>(position_goal_topic, 1000, std::bind(&Nav_node::goal_callback, this, _1));
    this->sub_robot_data = this->create_subscription<cdf_msgs::msg::RobotData>(robot_data_topic, 1000, std::bind(&Nav_node::robot_data_callback, this, _1));
    this->stop_sub = this->create_subscription<std_msgs::msg::Bool>(stop_topic, 1000, std::bind(&Nav_node::stop_callback, this, _1));

    // Load the map
    this->load_map_file(this->map_file);

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
    */
    
    for(int x = 0; x < MAP_WIDTH; x++){
        for(int y = 0; y < MAP_HEIGHT; y++){
            map[x][y] = map[x][y] || map2[x][y];
        }
    }
}

void Nav_node::make_circle_map(Circle obstacle, bool map[MAP_WIDTH][MAP_HEIGHT]){
    /*
    Create a map with a circle obstacle
    */
    float radius = obstacle.radius * 100;
    Point center = obstacle.center;
    center.x = center.x * 100;
    center.y = center.y * 100;

    for(int x = 0; x < MAP_WIDTH; x++){
        for(int y = 0; y < MAP_HEIGHT; y++){
            if (pow(x - center.x, 2) + pow(y - center.y, 2) < pow(radius, 2)){
                map[x][y] = true;
            }
            else{
                map[x][y] = false;
            }
        }
    }

}

void Nav_node::obstacle_disjunction(cdf_msgs::msg::MergedDataBis MergedData){
    /*
    This function will take the MergedDataBis message and extract the three obstacles
    */

    Circle obstacle[3];

    if (this->robot_number == 1){
        auto tmp = MergedData.robot_2.size();

        if (MergedData.robot_2[tmp - 1].position.x == 0 && MergedData.robot_2[tmp - 1].position.y == 0){
            // Robot 2 is not defined
            tmp = MergedData.ennemi_3.size();
            obstacle[0].center.x = MergedData.ennemi_3[tmp - 1].position.x;
            obstacle[0].center.y = MergedData.ennemi_3[tmp - 1].position.y;
        }
        else{
            // Robot 2 is defined
            tmp = MergedData.robot_2.size();
            obstacle[0].center.x = MergedData.robot_2[tmp - 1].position.x;
            obstacle[0].center.y = MergedData.robot_2[tmp - 1].position.y;
        }

    }
    else{
        auto tmp = MergedData.robot_1.size();
        if (MergedData.robot_1[tmp - 1].position.x == 0 && MergedData.robot_1[tmp - 1].position.y == 0){
            // Robot 1 is not defined
            auto tmp = MergedData.ennemi_3.size();
            obstacle[0].center.x = MergedData.ennemi_3[tmp - 1].position.x;
            obstacle[0].center.y = MergedData.ennemi_3[tmp - 1].position.y;
        }
        else{
            // Robot 1 is defined
            auto tmp = MergedData.robot_1.size();
            obstacle[0].center.x = MergedData.robot_1[tmp - 1].position.x;
            obstacle[0].center.y = MergedData.robot_1[tmp - 1].position.y;
        }
    }

    auto tmp = MergedData.ennemi_1.size();
    obstacle[1].center.x = MergedData.ennemi_1[tmp - 1].position.x;
    obstacle[1].center.y = MergedData.ennemi_1[tmp - 1].position.y;

    tmp = MergedData.ennemi_2.size();
    obstacle[2].center.x = MergedData.ennemi_2[tmp - 1].position.x;
    obstacle[2].center.y = MergedData.ennemi_2[tmp - 1].position.y;

    #ifndef WORLD_OF_SILENCE
    RCLCPP_INFO(this->get_logger(), "Obstacle 1 : (%d, %d)", obstacle[0].center.x, obstacle[0].center.y);
    RCLCPP_INFO(this->get_logger(), "Obstacle 2 : (%d, %d)", obstacle[1].center.x, obstacle[1].center.y);
    RCLCPP_INFO(this->get_logger(), "Obstacle 3 : (%d, %d)", obstacle[2].center.x, obstacle[2].center.y);
    #endif

    this->obstacle_processing(obstacle);
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
            if (abs(obstacle[i].radius - this->obstacles[j].radius) > this->variation_obs\
                && abs(obstacle[i].center.x - this->obstacles[j].center.x) > this->variation_obs\
                && abs(obstacle[i].center.y - this->obstacles[j].center.y) > this->variation_obs){
                variation = false;
                break;
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

        bool new_map[MAP_WIDTH][MAP_HEIGHT];
        for (int x = 0; x < MAP_WIDTH; x++){
            for (int y = 0; y < MAP_HEIGHT; y++){
                new_map[x][y] = this->base_map[x][y];
            }
        }
        bool temp_map[MAP_WIDTH][MAP_HEIGHT];
        // If there is a variation, update the obstacles
        for (int i = 0; i < 3; i++){
            this->obstacles[i] = obstacle[i];
            this->make_circle_map(obstacle[i], temp_map);
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
        + std::to_string(next_goal.x) 
        + " " + std::to_string(next_goal.y);
    
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
    std::string real_path = ament_index_cpp::get_package_share_directory("nav_node") + "/maps/" + map_file;
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

void Nav_node::robot_data_callback(const cdf_msgs::msg::RobotData msg){
    /*
    Get odometry information from the RobotData topic
    */
    
    // Update the robot data
    this->robot_data = msg;    

    // Update the robot position
    this->robot_position.x = (this->robot_data.position).x;
    this->robot_position.y = (this->robot_data.position).y;

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
    this->path = this->nav_alg.calculate_path(this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y);
    if (this->path.empty()){
        // If the path is empty, the goal is unreachable
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

    // If there is no path, compute one
    if (this->path.empty() && is_defined(this->robot_goal)){
        // Compute the path
        this->path = this->nav_alg.calculate_path(this->robot_position.x, this->robot_position.y, this->robot_goal.x, this->robot_goal.y);
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
