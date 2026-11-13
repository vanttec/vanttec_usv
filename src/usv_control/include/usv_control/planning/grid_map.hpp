#pragma once
#include <cstdint>
#include <vector>

struct Cell {
  int i, j;
  bool operator==(const Cell& other) const { return i == other.i && j == other.j; }
};

class GridMap {
public:
  GridMap(double origin_x, double origin_y, double resolution, int size_x, int size_y);

  bool worldToCell(double x, double y, int& i, int& j) const;
  void cellToWorld(int i, int j, double& x, double& y) const;
  bool isInside(int i, int j) const;

  uint8_t getCost(int i, int j) const;
  void    setCost(int i, int j, uint8_t cost);
  void fill(uint8_t value);

  int    sizeX() const;
  int    sizeY() const;
  double resolution() const;

  static constexpr uint8_t FREE   = 0;
  static constexpr uint8_t LETHAL = 254;

private:
  int index(int i, int j) const;
  double origin_x_, origin_y_, res_;
  int size_x_, size_y_;
  std::vector<uint8_t> data_;
};