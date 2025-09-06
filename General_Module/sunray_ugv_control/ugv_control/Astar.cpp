#include "Astar.h"

// 创建带森林区域的地图
GridWithWeights make_diagram4()
{
    GridWithWeights grid(10, 10);
    add_rect(grid, 1, 7, 4, 9);
    return grid;
}

int main()
{
    GridWithWeights grid = make_diagram4();
    GridLocation start{1, 4}, goal{8, 3};
    Astar astar(10, 10);
    astar.setGraph(&grid);
    astar.setStart(start.x, start.y);
    astar.setGoal(goal.x, goal.y);
    astar.a_star_search();
    std::vector<GridLocation> path = astar.reconstruct_path();
    for (const auto &loc : path)
    {
        std::cout << loc << " ";
    }
    std::cout << std::endl;
}