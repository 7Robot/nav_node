#ifndef PSTAR_HPP
#define PSTAR_HPP

#include "nav_node/nav_node.hpp"

struct Point{
    int x;
    int y;
    float theta;
};

class PStar{
    public:
        PStar();
        ~PStar();

        int calculate_path(int startX, int startY, int endX, int endY, std::vector<Point> &result_path);
        void set_map(bool map[MAP_WIDTH][MAP_HEIGHT]);

    private:
        int map_cost[MAP_WIDTH][MAP_HEIGHT];
        bool map[MAP_WIDTH][MAP_HEIGHT];
        bool get_cost(Point p);
};

#endif // PSTAR_HPP
