#include "usv_control/planning/costmap_builder.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

CostmapBuilder::CostmapBuilder(const InflationParams& params) : params_(params){}

void CostmapBuilder::build(GridMap& map, const std::vector<Obstacle>& obstacles) const {
    map.fill(GridMap::FREE);
    for(const auto& obstacle : obstacles){
        stampObstacle(map,obstacle);
    }
}

void CostmapBuilder::stampObstacle(GridMap& map, const Obstacle& obstacle) const{
    double reach = obstacle.radius + params_.influence_radius;
    int i_min, j_min; //Esquina inferior 
    int i_max, j_max; //Esquina superior
    map.worldToCell(obstacle.x - reach, obstacle.y - reach, i_min, j_min);
    map.worldToCell(obstacle.x + reach, obstacle.y + reach, i_max, j_max);
    i_min = std::clamp(i_min, 0, map.sizeX() - 1);
    j_min = std::clamp(j_min, 0, map.sizeY() - 1);
    i_max = std::clamp(i_max, 0, map.sizeX() - 1);
    j_max = std::clamp(j_max, 0, map.sizeY() - 1);

    double cx, cy;
    for(int i = i_min; i <= i_max; i++){
        for(int j = j_min; j <= j_max; j++){
            map.cellToWorld(i,j,cx,cy);
            double distance_to_center =std::hypot(cx-obstacle.x,cy-obstacle.y);
            double d = distance_to_center - obstacle.radius;
            uint8_t new_cost = costAtDistance(d);
            map.setCost(i,j,std::max(map.getCost(i,j),new_cost));
        }
    } 

}

uint8_t CostmapBuilder::costAtDistance(double d) const{
    if(d <= params_.lethal_radius){
        return GridMap::LETHAL;
    }
    else if(d <= params_.influence_radius){
        double k = params_.decay;
        double cost = 253 * std::exp(-k * (d - params_.lethal_radius));
        return static_cast<uint8_t>(std::max(1.0, cost));
    }
    return GridMap::FREE;
}
