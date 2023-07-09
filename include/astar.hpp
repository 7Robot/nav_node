#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "nav_node.hpp"
#define MAP_WIDTH 150
#define MAP_HEIGHT 100

struct Node{
    int x;
    int y;
    int parentX;
    int parentY;
    bool reachable;
    float gCost;
    float hCost;
    float fCost;
};

class Astar{
    public:
        Astar();
        ~Astar();

        std::vector<Node> calculate_path(int startX, int startY, int endX, int endY);
        void set_map(bool map[MAP_WIDTH][MAP_HEIGHT]);

    private:
        bool map[MAP_WIDTH][MAP_HEIGHT];
        float heuristic(Node current, Node end);
        bool is_valid(int x, int y);
};

#endif