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
    int tmp1 = current.x - end.x;
    int tmp2 = current.y - end.y;
    return sqrt(tmp1*tmp1 + tmp2*tmp2);
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

    int temp;

    while(openList.size() > 0){
        //Find the node with the lowest fCost
        float lowestCost = openList[0].fCost;
        int lowestCostIndex = 0;
        temp = openList.size();
        for(int i = 0; i < temp; i++){
            if(openList[i].fCost < lowestCost){
                lowestCost = openList[i].fCost;
                lowestCostIndex = i;
            }
        }

        Node currentNode = openList[lowestCostIndex];
        closedList.push_back(currentNode);
        openList.erase(openList.begin()+lowestCostIndex);
           

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
                        temp = openList.size();
                        for (int i=0; i<temp; i++){
                            if (openList[i].x == currentNode.x+x && openList[i].y == currentNode.y+y){
                                //Skip the neighbor if it is in the open list
                                verifier = false;
                                break;
                            }                
                        }
                        if (verifier){
                            // Verify if the neighbor is in the closed list
                            temp = closedList.size();
                            for (int i=0; i<temp; i++){
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
                                    neighbor.gCost = currentNode.gCost + 1.;
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
           }
    
        }
        
    }
    if (endFound){
        // Reconstruct the path
        std::cout << "Reconstructing path" << std::endl;
        std::vector<Node> path;
        Node currentNode = end;
        while (currentNode.x != startX || currentNode.y != startY){
            bool verifier = false;
            path.push_back(currentNode);
            //std::cout << "Searching for parent node"<< currentNode.parentX<< ", "<< currentNode.parentY << std::endl;
            temp = closedList.size();
            for (int i=0; i<temp; i++){
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
