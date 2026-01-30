#ifndef CATMUL_ROM_H
#define CATMUL_ROM_H

#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>

struct Segment {
    Eigen::Vector2d a, b, c, d;
};

class CatmulRom
{
public:
    CatmulRom();
    CatmulRom(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3);
    void update(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3);
    Eigen::Vector2d get_s(double t);
    Eigen::Vector2d get_s_dot(double t);
    void calc_arc_length();
    double get_arc_length(double a, double b);
    double closest_t(Eigen::Vector3d p);
    double get_la(double t, double D);
    static double distance(Eigen::Vector2d a, Eigen::Vector2d b);

    Segment s_;
    double L_;

private:
    double alpha_;
    double tension_;
};

#endif // CATMUL_ROM_H
