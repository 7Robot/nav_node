#include "nav_node.hpp"

PStar::PStar(){
    //Constructor
}

PStar::~PStar(){
    //Destructor
}

void PStar::set_map(bool map[MAP_WIDTH][MAP_HEIGHT]){
    //Set the map
    for(int x = 0; x < MAP_WIDTH; x++){
        for(int y = 0; y < MAP_HEIGHT; y++){
            this->map[x][y] = map[x][y];
            this->map_cost[x][y] = INT_MAX;            
        }
    }
}

bool PStar::get_cost(Point p){
    //Get the cost of a node
    if (this->map[p.x][p.y] == false){
        return false;
    }

    int old_cost = this->map_cost[p.x][p.y];
    int cost = INT_MAX;
    for (int dx = -1; dx <= 1; dx++){
        for (int dy = -1; dy <= 1; dy++){
            if (dx == 0 || dy == 0){
                cost = this->map_cost[p.x + dx][p.y + dy] + 10;
            }
            else{
                cost = this->map_cost[p.x + dx][p.y + dy] + 14;
            }
            if (cost < this->map_cost[p.x][p.y]){
                this->map_cost[p.x][p.y] = cost;
            }
        }
    }
    return old_cost != this->map_cost[p.x][p.y];
}

std::vector<Point> PStar::calculate_path(int startX, int startY, int endX, int endY){
    Point tmp;
    bool running = true;
    int count = 0;

    bool still_running = true;

    this->map_cost[startX][startY] = 0;

    while (running){
        //std::cout << "Calculating cost... " << count << std::endl;
        count = 0;
        running = false;
        for (int x = 1; x < MAP_WIDTH-1; x++){
            for (int y = 1; y < MAP_HEIGHT-1; y++){
                tmp.x = x;
                tmp.y = y;
                still_running = this->get_cost(tmp);
                running = running || still_running;
                if (still_running){count ++;}
            }
        }
        if (!running){continue;}
        for (int x = MAP_WIDTH-2; x > 1; x--){
            for (int y = MAP_HEIGHT-2; y > 1; y--){
                tmp.x = x;
                tmp.y = y;
                still_running = this->get_cost(tmp);
                running = running || still_running;
                if (still_running){count ++;}
            }
        }
    }
    //std::cout << "Cost calculated" << std::endl;
    std::vector<Point> path;

    Point current;
    current.x = endX;
    current.y = endY;
    if (this->map_cost[current.x][current.y] == INT_MAX){
        //std::cout << "No path found" << std::endl;
        return path;
    }
    path.push_back(current);
    while (current.x != startX || current.y != startY){
        int best_cost = INT_MAX;
        Point best_point;
        for (int dx = -1; dx <= 1; dx++){
            for (int dy = -1; dy <= 1; dy++){
                tmp.x = current.x + dx;
                tmp.y = current.y + dy;
                if (tmp.x < 0 || tmp.x >= MAP_WIDTH || tmp.y < 0 || tmp.y >= MAP_HEIGHT){
                    continue;
                }
                if (this->map_cost[tmp.x][tmp.y] < best_cost){
                    best_cost = this->map_cost[tmp.x][tmp.y];
                    best_point = tmp;
                }
            }
        }
        current = best_point;
        path.push_back(current);
    }
    //std::cout << "Path calculated" << std::endl;
    return path;
}

