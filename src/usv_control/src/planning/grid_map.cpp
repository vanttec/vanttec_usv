#include "usv_control/planning/grid_map.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

GridMap::GridMap(double origin_x, double origin_y, double resolution, int size_x, int size_y) : 
origin_x_(origin_x), origin_y_(origin_y), res_(resolution), size_x_(size_x), size_y_(size_y)
{
  if (resolution <= 0.0 || size_x <= 0 || size_y <= 0) {
    throw std::invalid_argument("GridMap: resolución y tamaños deben ser > 0");
  }
  data_.assign(size_x * size_y, FREE);
} 

bool GridMap::worldToCell(double x, double y, int& i, int& j) const {
    i = static_cast<int>(std::floor((x - origin_x_) / res_));
    j = static_cast<int>(std::floor((y - origin_y_) / res_));
    return isInside(i, j);
}

void GridMap::cellToWorld(int i, int j, double& x, double& y) const {
    x =  (i + 0.5) * res_ + origin_x_;
    y =  (j + 0.5) * res_ + origin_y_;
}

bool GridMap::isInside(int i, int j) const {
  return (i >= 0 && i < size_x_) && (j >= 0 && j < size_y_) ;
}

uint8_t GridMap::getCost(int i, int j) const {
  if (!isInside(i, j)) {
    return LETHAL;
  }
  return data_[index(i, j)];
}

void GridMap::setCost(int i, int j, uint8_t cost) {
  if (!isInside(i, j)){
    return;
  }
  data_[index(i, j)] = cost;
}

void GridMap::fill(uint8_t value){
  std::fill(data_.begin(),data_.end(),value);
}

int GridMap::sizeX() const { 
  return size_x_; 
}
int GridMap::sizeY() const { 
  return size_y_; 
}
double GridMap::resolution() const { 
  return res_; 
}


int GridMap::index(int i, int j) const {
  return i * size_y_ + j;
}