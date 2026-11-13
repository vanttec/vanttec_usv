#include "usv_control/planning/a_star_planner.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <queue>
#include <limits>
#include <functional>


AStarPlanner::AStarPlanner(double cost_weight): cost_weight_(cost_weight) {
    if (cost_weight_ < 0.0) {
        throw std::invalid_argument("AStarPlanner: cost_weight debe ser >= 0");
    }
}

int AStarPlanner::toId(const Cell& c, int size_y) const {
    return c.i * size_y + c.j;
}

Cell AStarPlanner::fromId(int id, int size_y) const {
    return Cell{id / size_y, id % size_y};
}

double AStarPlanner::heuristic(const Cell& a, const Cell& b, double res) const{
    return res * std::hypot(a.i - b.i, a.j - b.j);
}

double AStarPlanner::stepCost(const GridMap& map, const Cell& to, bool diagonal) const{
    double base = diagonal ? map.resolution() * std::sqrt(2.0) : map.resolution();
    return base  * (1.0 + cost_weight_ * map.getCost(to.i,to.j) / 253.0);
}

std::vector<Cell> AStarPlanner::reconstruct(const std::vector<int>& parent, int goal_id, int size_y) const{
    std::vector<Cell> path;
    int id = goal_id;
    while(id != -1){
        path.push_back(fromId(id,size_y));
        id = parent[id];
    }
    std::reverse(path.begin(),path.end());
    return path;
}

// El inicio siempre se acepta, aunque sea letal (decisión de diseño: si el viento
// empuja al bote cerca de una boya, congelarlo es lo peor). Pero si TODAS sus vecinas
// también son letales, A* no puede expandir nada. Por eso: primero se "escapa" en línea
// recta a la celda libre más cercana y desde ahí se planea normal.
std::optional<std::vector<Cell>> AStarPlanner::plan(const GridMap& map, const Cell& start, const Cell& goal) const {
    if (!map.isInside(start.i, start.j)) return std::nullopt;
    if (map.getCost(start.i, start.j) != GridMap::LETHAL) {
        return search(map, start, goal);
    }
    auto exit = nearestFree(map, start);
    if (!exit) return std::nullopt;
    auto rest = search(map, *exit, goal);
    if (!rest) return std::nullopt;
    rest->insert(rest->begin(), start);
    return rest;
}

std::optional<Cell> AStarPlanner::nearestFree(const GridMap& map, const Cell& start) const {
    const int sy = map.sizeY();
    std::vector<bool> seen(map.sizeX() * sy, false);
    std::queue<Cell> frontier;
    frontier.push(start);
    seen[toId(start, sy)] = true;
    const int di[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dj[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
    while (!frontier.empty()) {
        Cell c = frontier.front();
        frontier.pop();
        if (map.getCost(c.i, c.j) != GridMap::LETHAL) return c;
        for (int k = 0; k < 8; k++) {
            Cell n{c.i + di[k], c.j + dj[k]};
            if (!map.isInside(n.i, n.j)) continue;
            int id = toId(n, sy);
            if (seen[id]) continue;
            seen[id] = true;
            frontier.push(n);
        }
    }
    return std::nullopt;
}

std::optional<std::vector<Cell>> AStarPlanner::search(const GridMap& map, const Cell& start, const Cell& goal) const {
    if(!map.isInside(goal.i,goal.j) || map.getCost(goal.i,goal.j) == GridMap::LETHAL){
        return std::nullopt;
    }
    if(!map.isInside(start.i,start.j)){
        return std::nullopt;
    }
    if(start == goal){
        return std::vector<Cell>{start};
    }
    
    int sy = map.sizeY();
    int n = map.sizeX() * sy;
    double res = map.resolution();
    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n,-1);
    std::vector<bool> closed(n,false);
    int start_id = toId(start,sy), goal_id = toId(goal,sy);
    g[start_id] = 0.0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> open;
    open.push({heuristic(start,goal,res),start_id});

    const int di[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dj[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

    while(!open.empty()){
        int id = open.top().second;
        open.pop();
        if(closed[id]) continue;
        if(id == goal_id){
            return reconstruct(parent,goal_id,sy);
        }
        closed[id] = true;
        Cell actual = fromId(id,sy);
        for(int k = 0; k <= 7; k++){
            Cell neighbor = {actual.i + di[k], actual.j + dj[k]};
            if(!map.isInside(neighbor.i,neighbor.j)) continue;
            if(map.getCost(neighbor.i,neighbor.j) == GridMap::LETHAL) continue;
            int neighbor_id = toId(neighbor,sy);
            if(closed[neighbor_id]) continue;
            bool diagonal = (di[k] != 0 && dj[k] != 0);
            if(diagonal && (map.getCost(actual.i + di[k], actual.j) == GridMap::LETHAL || map.getCost(actual.i, actual.j + dj[k]) == GridMap::LETHAL)) continue;
            double g_new = g[id] + stepCost(map,neighbor,diagonal);
            if(g_new < g[neighbor_id]){
                g[neighbor_id] = g_new;
                parent[neighbor_id] = id;
                open.push({g_new + heuristic(neighbor,goal,res), neighbor_id});
            }
        }
    }
    return std::nullopt;
}