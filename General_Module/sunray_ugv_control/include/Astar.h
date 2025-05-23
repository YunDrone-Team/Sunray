#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>
#include "printf_format.h"

using namespace sunray_logger;

// 简单图结构 (SimpleGraph)
// 使用哈希表存储图的边关系
// edges：键为节点ID，值为相邻节点列表
// neighbors()：获取某节点的所有相邻节点
struct SimpleGraph
{
    std::unordered_map<char, std::vector<char>> edges;

    std::vector<char> neighbors(char id)
    {
        return edges[id];
    }
};

// 网格位置 (GridLocation)
// 包含x,y坐标
// 实现了比较运算符和哈希函数
// 可以放入无序集合(unordered_set)中
struct GridLocation
{
    int x, y;
};

namespace std
{
    /* implement hash function so we can put GridLocation into an unordered_set */
    template <>
    struct hash<GridLocation>
    {
        std::size_t operator()(const GridLocation &id) const noexcept
        {
            // I wish built-in std::hash worked on pair and tuple
            return std::hash<int>()(id.x ^ (id.y << 16));
        }
    };
}

// 方形网格 (SquareGrid)
// 定义四个移动方向(东、西、南、北)
// 检查位置是否在边界内(in_bounds)
// 检查位置是否可通过(passable)
// 获取相邻位置(neighbors)
// 特殊处理：偶数位置反转邻居顺序避免路径偏向
struct SquareGrid
{
    // static std::array<GridLocation, 4> DIRS;
    static std::array<GridLocation, 8> DIRS;

    int width, height;
    // 存储墙壁位置
    std::unordered_set<GridLocation> walls;

    SquareGrid(int width_, int height_)
        : width(width_), height(height_) {}

    bool in_bounds(GridLocation id) const
    {
        // if( id.x < 0 || id.x >= width || id.y < 0 || id.y >= height)
        // {
        //     std::cout << "map width: "<< width << " height: " << height << std::endl;
        //     std::cout<< "out of bounds: " << id.x << " " << id.y << std::endl;
        // }
        return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
    }

    void print_walls() const
    {
        std::cout << "walls: " << std::endl;
        Logger::print_color(int(LogColor::white), "walls: ");
        for (auto wall : walls)
        {
            // std::cout << wall.x << " " << wall.y << std::endl;
            Logger::print_color(int(LogColor::white), std::to_string(wall.x) + " " + std::to_string(wall.y));
        }
    }

    // 清除墙壁
    void clear_walls()
    {
        walls.clear();
    }

    /* 检查位置是否可通过 */
    bool passable(GridLocation id) const
    {
        // print_walls();
        return walls.find(id) == walls.end();
    }

    std::vector<GridLocation> neighbors(GridLocation id) const
    {
        std::vector<GridLocation> results;

        for (GridLocation dir : DIRS)
        {
            GridLocation next{id.x + dir.x, id.y + dir.y};
            if (in_bounds(next) && passable(next))
            {
                results.push_back(next);
            }
        }

        if ((id.x + id.y) % 2 == 0)
        {
            // see "Ugly paths" section for an explanation:
            std::reverse(results.begin(), results.end());
        }

        return results;
    }
};

// std::array<GridLocation, 4> SquareGrid::DIRS = {
//     /* East, West, North, South */
//     GridLocation{1, 0}, GridLocation{-1, 0},
//     GridLocation{0, -1}, GridLocation{0, 1}};

std::array<GridLocation, 8> SquareGrid::DIRS = {
    /* N， S， E， W, NE， NW， SE， SW */
    GridLocation{0, -1}, GridLocation{0, 1},
    GridLocation{1, 0}, GridLocation{-1, 0},
    GridLocation{1, -1}, GridLocation{-1, -1},
    GridLocation{1, 1}, GridLocation{-1, 1}};

// Helpers for GridLocation

bool operator==(GridLocation a, GridLocation b)
{
    return a.x == b.x && a.y == b.y;
}

bool operator!=(GridLocation a, GridLocation b)
{
    return !(a == b);
}

bool operator<(GridLocation a, GridLocation b)
{
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream &operator<<(std::basic_iostream<char>::basic_ostream &out, const GridLocation &loc)
{
    out << '(' << loc.x << ',' << loc.y << ')';
    return out;
}

// 在指定矩形区域添加墙壁
void add_rect(SquareGrid &grid, int x1, int y1, int x2, int y2)
{
    for (int x = x1; x < x2; ++x)
    {
        for (int y = y1; y < y2; ++y)
        {
            grid.walls.insert(GridLocation{x, y});
        }
    }
}

void add_wall(SquareGrid &grid, int x, int y)
{
    grid.walls.insert(GridLocation{x, y});
}

void remove_wall(SquareGrid &grid, int x, int y)
{
    grid.walls.erase(GridLocation{x, y});
}

// 带权网格 (GridWithWeights)
// 继承SquareGrid
// 添加森林区域(移动成本为5)
// cost()函数计算移动代价
struct GridWithWeights : SquareGrid
{
    GridWithWeights(int w, int h) : SquareGrid(w, h) {}
    double cost() const
    {
        return 1;
    }
};

// 优先队列 (PriorityQueue)
// 基于标准库的优先队列实现
// 支持放入元素和优先级、取出最小元素
template <typename T, typename priority_t>
struct PriorityQueue
{
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
                        std::greater<PQElement>>
        elements;

    inline bool empty() const
    {
        return elements.empty();
    }

    inline void put(T item, priority_t priority)
    {
        elements.emplace(priority, item);
    }

    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

class Astar
{
public:
    Astar() {};
    Astar(int width, int height)
    {
        graph = new GridWithWeights(width, height);
    };
    ~Astar()
    {
        delete graph;
    };
    void init(int width, int height)
    {
        graph = new GridWithWeights(width, height);
    }
    void setStart(int x, int y)
    {
        // std::cout << "set start: " << x << " " << y << std::endl;
        Logger::print_color(int(LogColor::white), "set start: " + std::to_string(x) + std::to_string(y));
        start_set = true;
        start.x = x;
        start.y = y;
    }
    void setGoal(int x, int y)
    {
        // std::cout << "set goal: " << x << " " << y << std::endl;
        Logger::print_color(int(LogColor::white), "set goal: " + std::to_string(x) + std::to_string(y));
        goal_set = true;
        goal.x = x;
        goal.y = y;
    }
    void setGraph(GridWithWeights *graph)
    {
        // 浅拷贝
        *this->graph = *graph;
    }

    // 启发函数 使用曼哈顿距离
    inline double heuristic(GridLocation a, GridLocation b)
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    // A*搜索算法
    void a_star_search();

    // 路径重构
    std::vector<GridLocation> reconstruct_path();

private:
    bool goal_set = false;
    bool start_set = false;
    bool path_found = false;
    GridWithWeights *graph;   // 地图
    GridLocation start, goal; // 起点和终点

    std::unordered_map<GridLocation, GridLocation> came_from;
    std::unordered_map<GridLocation, double> cost_so_far;
};

// 路径重构
std::vector<GridLocation> Astar::reconstruct_path()
{
    std::vector<GridLocation> path;
    GridLocation current = goal;
    if (came_from.find(goal) == came_from.end())
    {
        return path; // no path can be found
    }
    while (current != start)
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    return path;
}

void Astar::a_star_search()
{

    // 重置 came_from和cost_so_far
    came_from.clear();
    cost_so_far.clear();
    PriorityQueue<GridLocation, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;
    // std::cout << "start search!" << std::endl;
    Logger::print_color(int(LogColor::white), "start search!");

    while (!frontier.empty())
    {
        GridLocation current = frontier.get();

        if (current == goal)
        {
            break;
        }

        for (GridLocation next : graph->neighbors(current))
        {
            double new_cost = cost_so_far[current] + graph->cost();
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;
                double priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
    // 添加路径失败判断
    if (came_from.find(goal) == came_from.end()) {
        std::cout << "Path planning failed: No path exists from start to goal." << std::endl;
        Logger::print_color(int(LogColor::white), "Path planning failed: No path exists from start to goal.");
        path_found = false;
    } else {
        path_found = true;
    }
}

#endif // Astar_h