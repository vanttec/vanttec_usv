#include "usv_control/planning/path_post_processor.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <stdexcept>

PathPostProcessor::PathPostProcessor(uint8_t cost_threshold) : cost_threshold_(cost_threshold) {
  if (cost_threshold >= GridMap::LETHAL) {
    throw std::invalid_argument("PathPostProcessor: umbral debe ser < LETHAL");
  }
}

bool PathPostProcessor::lineOfSight(const GridMap& map, const Cell& a, const Cell& b) const{
   int di = b.i - a.i;
   int dj = b.j - a.j;
   int steps = std::max(std::abs(di), std::abs(dj));

   for(int s = 1; s <= steps; ++s){
    double t = static_cast<double>(s) / steps;
    int i = static_cast<int>(std::lround(a.i + t * di));
    int j = static_cast<int>(std::lround(a.j + t * dj));
    if(map.getCost(i,j) > cost_threshold_) return false;
   }
   return true;
}

std::vector<Cell> PathPostProcessor::shortcut(const GridMap& map, const std::vector<Cell>& path) const {
    if(path.size() <= 2) return path;

    std::vector<Cell> result{path[0]};
    size_t from = 0;
    while(from < path.size() - 1){
        size_t to = path.size() -1;
        while(to > from + 1 && !lineOfSight(map, path[from], path[to])){
            to--;
        }
        result.push_back(path[to]);
        from = to;
    }
    return result;
}
