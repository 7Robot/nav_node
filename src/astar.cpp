#include "nav_node.hpp"

Astar::Astar(){
    //Constructor
}

Astar::~Astar(){
    //Destructor
}

void Astar::set_map(bool map[MAP_WIDTH][MAP_HEIGHT]){
    //Set the map
    for(int x = 0; x < MAP_WIDTH; x++){
        for(int y = 0; y < MAP_HEIGHT; y++){
            this->map[x][y] = map[x][y];
        }
    }
}

float Astar::heuristic(Node current, Node end){
    //Calculate the heuristic
    return sqrt(pow(current.x - end.x, 2) + pow(current.y - end.y, 2));
}

bool Astar::is_valid(int x, int y){
    //Verify if the node is valid
    if(x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT){
        return false;
    }
    else{
        return map[x][y];
    }
}

std::vector<Node> Astar::calculate_path(int startX, int startY, int endX, int endY){
    //Calculate the path
    Node start;
    start.x = startX;
    start.y = startY;
    start.parentX = startX;
    start.parentY = startY;
    start.gCost = 0;
    start.hCost = 0;
    start.fCost = 0;

    Node end;
    end.x = endX;
    end.y = endY;
    end.parentX = endX;
    end.parentY = endY;
    end.gCost = 0;
    end.hCost = 0;
    end.fCost = 0;

    std::vector<Node> openList;
    std::vector<Node> closedList;
    openList.push_back(start);

    bool endFound = false;

    while(openList.size() > 0){
        //Find the node with the lowest fCost
        float lowestCost = openList[0].fCost;
        int lowestCostIndex = 0;
        for(int i = 0; i < openList.size(); i++){
            if(openList[i].fCost < lowestCost){
                lowestCost = openList[i].fCost;
                lowestCostIndex = i;
            }
        }

        Node currentNode = openList[lowestCostIndex];
        if(currentNode.x == end.x && currentNode.y == end.y){
            //We have reached the end
            //std::cout << "Path found" << std::endl;
            end.parentX = currentNode.parentX;
            end.parentY = currentNode.parentY;
            endFound = true;
            break;
        }
        else{
            //std::cout << "Current node: " << currentNode.x << ", " << currentNode.y << std::endl;
            // Get all the neighbors
            for (int x=-1; x<=1; x++){
                for (int y=-1; y<=1; y++){
                    if (x==0 && y==0){
                        //Skip the current node
                        continue;
                    }
                    else{
                        bool verifier = true;
                        // Verify if the neighbor is already in the open list
                        for (int i=0; i<openList.size(); i++){
                            if (openList[i].x == currentNode.x+x && openList[i].y == currentNode.y+y){
                                //Skip the neighbor if it is in the open list
                                verifier = false;
                                break;
                            }                
                        }
                        if (verifier){
                            // Verify if the neighbor is in the closed list
                            for (int i=0; i<closedList.size(); i++){
                                if (closedList[i].x == currentNode.x+x && closedList[i].y == currentNode.y+y){
                                    //Skip the neighbor if it is in the closed list
                                    verifier = false;
                                    break;
                                }                
                            }
                        }
                        if (verifier){
                            // Verify if the neighbor is in the map
                            if (is_valid(currentNode.x+x, currentNode.y+y)){

                                // Calculate the gCost, hCost and fCost
                                Node neighbor;
                                neighbor.x = currentNode.x+x;
                                neighbor.y = currentNode.y+y;
                                neighbor.parentX = currentNode.x;
                                neighbor.parentY = currentNode.y;
                                if (x==0 || y==0){
                                    neighbor.gCost = currentNode.gCost + 1;
                                }
                                else{
                                    neighbor.gCost = currentNode.gCost + 1.414;
                                }
                                neighbor.hCost = heuristic(neighbor, end);
                                neighbor.fCost = neighbor.gCost + neighbor.hCost;
                                // Add the neighbor to the open list
                                openList.push_back(neighbor);
                            }
                        }
                    }
            }
            //std::cout << "Open list size: " << openList.size() << std::endl;
            // Add the current node to the closed list
            closedList.push_back(currentNode);
            // Remove the current node from the open list
            openList.erase(openList.begin()+lowestCostIndex);
            //std::cout << "Changed size to " << openList.size() << std::endl;

        }
    
        }
        
    }
    if (endFound){
        // Reconstruct the path
        std::vector<Node> path;
        Node currentNode = end;
        while (currentNode.x != startX || currentNode.y != startY){
            bool verifier = false;
            path.push_back(currentNode);
            //std::cout << "Searching for parent node"<< currentNode.parentX<< ", "<< currentNode.parentY << std::endl;
            for (int i=0; i<closedList.size(); i++){
                if (closedList[i].x == currentNode.parentX && closedList[i].y == currentNode.parentY){
                    currentNode = closedList[i];
                    //std::cout << "Found parent node: " << currentNode.x << ", " << currentNode.y << std::endl;
                    verifier = true;
                    break;
                }
            }
            if (!verifier){
                //std::cout << "Parent node not found" << std::endl;
                exit(1);
            }
        }
        path.push_back(start);
        
        return path;
    }
    else{
        std::vector<Node> path;
        return path;
    }
}
