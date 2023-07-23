#include "nav_node/nav_node.hpp"

/*
This file contains the implementation of the P* algorithm
*/


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
    /*
    This function assign the cost to reach a node p 
    regarding the 8 node around it.

    Return true if the cost has changed
    */
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
            if (cost < this->map_cost[p.x][p.y] && cost > 0){
                this->map_cost[p.x][p.y] = cost;
            }
        }
    }
    return old_cost != this->map_cost[p.x][p.y];
}

int PStar::calculate_path(int startX, int startY, int endX, int endY, std::vector<Point> &result_path){
    /*
    This function loop until the cost of all the nodes 
    can't change anymore.

    Ultimately it will calculate the best path from the
    start point to the end point and return it.
    */
    
    Point tmp;
    bool running = true;
    int count = 0;

    bool still_running = true;

    this->map_cost[startX][startY] = 0;
    // Errno -2 Start in obstacle
    if (startX < 0 || startY < 0 || startX > MAP_WIDTH || startY > MAP_HEIGHT || this->map[startX][startY] == false){return -2;}
    // Errno -3 End in obstacle
    if (endX < 0 || endY < 0 || endX > MAP_WIDTH || endY > MAP_HEIGHT || this->map[endX][endY] == false){return -3;}


    while (running){
        
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
        return -1;
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
    // Rewrite the path in the parameter
    int length = path.size();
    for (int i = 0; i < length ; i++){
        result_path.push_back(path[i]);
    }

    return 0;
}

