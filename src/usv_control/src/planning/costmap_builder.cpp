#include "usv_control/planning/costmap_builder.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>

CostmapBuilder::CostmapBuilder(const InflationParams& params) : params_(params){}

void CostmapBuilder::build(GridMap& map, const std::vector<Obstacle>& obstacles) const {
    map.fill(GridMap::FREE);
    stamp(map, obstacles);
}

void CostmapBuilder::stamp(GridMap& map, const std::vector<Obstacle>& obstacles) const {
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

void CostmapBuilder::stampGeofence(GridMap& map, const std::vector<Point2D>& polygon) const {
    if (polygon.size() < 3) return;  // sin geocerca
    double cx, cy;
    for (int i = 0; i < map.sizeX(); i++) {
        for (int j = 0; j < map.sizeY(); j++) {
            map.cellToWorld(i, j, cx, cy);
            uint8_t new_cost = insidePolygon(polygon, cx, cy)
                ? costAtDistance(distanceToEdges(polygon, cx, cy))
                : GridMap::LETHAL;
            map.setCost(i, j, std::max(map.getCost(i, j), new_cost));
        }
    }
}

// Ray casting: lanza un rayo hacia +x y cuenta cuántas aristas cruza.
// Impar = dentro, par = fuera.
bool CostmapBuilder::insidePolygon(const std::vector<Point2D>& poly, double x, double y) {
    bool inside = false;
    for (size_t a = 0, b = poly.size() - 1; a < poly.size(); b = a++) {
        const Point2D& p = poly[a];
        const Point2D& q = poly[b];
        bool crosses_y = (p.y > y) != (q.y > y);
        if (crosses_y) {
            double x_cross = p.x + (y - p.y) * (q.x - p.x) / (q.y - p.y);
            if (x < x_cross) inside = !inside;
        }
    }
    return inside;
}

// Distancia mínima del punto a cualquier arista (segmento) del polígono.
double CostmapBuilder::distanceToEdges(const std::vector<Point2D>& poly, double x, double y) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t a = 0, b = poly.size() - 1; a < poly.size(); b = a++) {
        const Point2D& p = poly[b];
        const Point2D& q = poly[a];
        double ex = q.x - p.x, ey = q.y - p.y;
        double len2 = ex * ex + ey * ey;
        // t = proyección del punto sobre el segmento, recortada a [0,1]
        double t = len2 > 0.0 ? std::clamp(((x - p.x) * ex + (y - p.y) * ey) / len2, 0.0, 1.0) : 0.0;
        best = std::min(best, std::hypot(x - (p.x + t * ex), y - (p.y + t * ey)));
    }
    return best;
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
