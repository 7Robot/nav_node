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
		// (x,y) is a wall
        return false;
    }

    int old_cost = this->map_cost[p.x][p.y];
    int cost = INT_MAX; // Set cost to +inf

	// We will browse every 8 point around
    for (int dx = -1; dx <= 1; dx++){
        for (int dy = -1; dy <= 1; dy++){
            if (dx == 0 || dy == 0){
                cost = this->map_cost[p.x + dx][p.y + dy] + 10; // 10 is 10*sqrt(dx^2+dy^2)
            }
            else{
                cost = this->map_cost[p.x + dx][p.y + dy] + 14; //14 is a simplification of 10*sqrt(dx^2+dy^2)
            }
            if (cost < this->map_cost[p.x][p.y] && cost > 0){
				// We assign the result if we found a better way to go to (x,y)
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
    
    Point tmp; // A temporary, useless variable to store result temporarly
    bool running = true;
    bool still_running = true;

    this->map_cost[startX][startY] = 0;
    // Errno -2 Start in obstacle
    if (startX < 0 || startY < 0 || startX > MAP_WIDTH || startY > MAP_HEIGHT || this->map[startX][startY] == false){return -2;}
    // Errno -3 End in obstacle
    if (endX < 0 || endY < 0 || endX > MAP_WIDTH || endY > MAP_HEIGHT || this->map[endX][endY] == false){return -3;}


    while (running){
        
        running = false;
        for (int x = 1; x < MAP_WIDTH-1; x++){// Browse the map forward...
            for (int y = 1; y < MAP_HEIGHT-1; y++){
                tmp.x = x;
                tmp.y = y;
                still_running = this->get_cost(tmp);
                running = running || still_running; // False only if every iteration is false
            }
        }
        if (!running){continue;} 
        for (int x = MAP_WIDTH-2; x > 1; x--){// ...and backward.
            for (int y = MAP_HEIGHT-2; y > 1; y--){
                tmp.x = x;
                tmp.y = y;
                still_running = this->get_cost(tmp);
                running = running || still_running;
            }
        }
    }
	
	// Rebuild the path starting from the end
    std::vector<Point> path;

    Point current;
    current.x = endX;
    current.y = endY;
    if (this->map_cost[current.x][current.y] == INT_MAX){
        return -1; // Errno -1 path not found
    }
    path.push_back(current);
    while (current.x != startX || current.y != startY){
        int best_cost = INT_MAX;
        Point best_point;

        for (int dx = -1; dx <= 1; dx++){ // For each neighbour find the best cost (the nearest point from start)
            for (int dy = -1; dy <= 1; dy++){
                tmp.x = current.x + dx;
                tmp.y = current.y + dy;
                if (tmp.x < 0 || tmp.x >= MAP_WIDTH || tmp.y < 0 || tmp.y >= MAP_HEIGHT){
					// If the point is out of the map, stop considering this point (or it may segfault)
                    continue;
                }
                if (this->map_cost[tmp.x][tmp.y] < best_cost){
                    best_cost = this->map_cost[tmp.x][tmp.y];
                    best_point = tmp;
                }
            }
        }
		// Add the best neighbour to the path and restart searching from it
        current = best_point;
        path.push_back(current);
    }

    // Rewrite the path in the vector given in parameter
    int length = path.size();
    for (int i = 0; i < length ; i++){
        result_path.push_back(path[i]);
    }

    return 0;
}

