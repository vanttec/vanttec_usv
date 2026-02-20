#include "CatmulRom.h"

CatmulRom::CatmulRom(){
    alpha_ = 1.0;
    tension_ = 0.2;
}

CatmulRom::CatmulRom(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3) : CatmulRom() {
    update(p0, p1, p2, p3);
}

void CatmulRom::update(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3){
    double t01 = pow(distance(p0,p1),alpha_);
    double t12 = pow(distance(p1,p2),alpha_);
    double t23 = pow(distance(p2,p3),alpha_);

    Eigen::Vector2d m1, m2;
    m1 = (1.0 - tension_) * (p2 - p1 + t12 * ((p1 - p0) / t01 - (p2 - p0) / (t01 + t12)));
    m2 = (1.0 - tension_) * (p2 - p1 + t12 * ((p3 - p2) / t23 - (p3 - p1) / (t12 + t23)));

    s_.a = 2.0 * (p1 - p2) + m1 + m2;
    s_.b = -3.0 * (p1 - p2) - m1 - m1 - m2;
    s_.c = m1;
    s_.d = p1;

    calc_arc_length();
}

double CatmulRom::distance(Eigen::Vector2d a, Eigen::Vector2d b){
    return sqrt((a(0)-b(0))*(a(0)-b(0))+(a(1)-b(1))*(a(1)-b(1)));
}

Eigen::Vector2d CatmulRom::get_s(double t){
    return s_.a * t * t * t +
        s_.b * t * t +
        s_.c * t +
        s_.d;
}

Eigen::Vector2d CatmulRom::get_s_dot(double t){
    return 3 * s_.a * t * t +
        2 * s_.b * t +
        s_.c;
}

double CatmulRom::closest_t(Eigen::Vector3d p3){
    Eigen::Vector2d p{p3.x(),p3.y()};
    // s = at^3+bt^2+ct+d
    // s_dot = 3at^2+2bt+c

    // Distance function derivative to find closest point in s
    // D = ||s-p||^2
    // d = 2(s-p)s_dot = 0
    // (s-p)*(s_dot) = 0
    // (at^3+bt^2+ct+d-p).dot(3at^2+2bt+c)
    // = 3a^2t^5+2abt^4+act^3+3abt^4+2b^2t^3+bct^2+3act^3+2bct^2+c^2t+3a(d-p)t^2+2b(d-p)t+c(d-p)
    // = (3a^2)t^5+(2ab+3ab)t^4+(ac+2b^2+3ac)t^3+(bc+2bc+3a(d-p))t^2+(c^2+2b(d-p))t+c(d-p)
    // = (3a^2)t^5+(5ab)t^4+(4ac+2b^2)t^3+(3bc+3a(d-p))t^2+(c^2+2b(d-p))t+c(d-p) = 0
    
    // Formulate 5th degree polynomial (coefficients from lowest to highest degree)
    Eigen::VectorXd coefficients(6);
    coefficients <<
        s_.c.dot(s_.d-p),
        s_.c.dot(s_.c) + 2*s_.b.dot(s_.d-p),
        3*s_.b.dot(s_.c) + 3*s_.a.dot(s_.d-p),
        4*s_.a.dot(s_.c) + 2*s_.b.dot(s_.b),
        5*s_.a.dot(s_.b),
        3*s_.a.dot(s_.a);

    // Get roots
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coefficients);
    const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType &r = solver.roots();

    // std::cout << "Found " << r.size() << " roots" << std::endl;
    // for(int i = 0 ; i < r.size() ; i++){
    //     std::cout << "Root " << i << ": " << r[i] << std::endl;
    // }

    // Evaluate dist. at limits
    double closest_t = 0;
    double min_dist = distance(p, get_s(0));
    if(distance(p,get_s(1)) < min_dist){
        min_dist = distance(p,get_s(1));
        closest_t = 1;
    }

    // Evaluate dist. at roots
    for(int i = 0 ; i < r.size() ; i++){
        // If the root is non-complex and [0,1] (in spline's t domain)
        if(r[i].imag() < 1e-9 && r[i].real() <= 1.0 && r[i].real() >= 0){
            double dist = distance(p, get_s(r[i].real()));
            if(dist < min_dist){
                min_dist = dist;
                closest_t = r[i].real();
            }
        }
    }

    return closest_t;
}

double CatmulRom::get_la(double t, double dist) {
    // We want to find u such that: ArcLength(t, t+u) = dist
    // Function to minimize: f(u) = ArcLength(t, t+u) - dist
    // Derivative: f'(u) = ||s_dot(t+u)|| (This is just speed!)

    double u = dist / L_; // Initial guess: Linear approximation
    double u_max = 1.0 - t; // Don't go past the end of the spline

    // Newton-Raphson iterations
    for(int i = 0; i < 5; i++) { // 5 iterations
        // Clamp u to valid range [0, 1-t]
        if (u < 0) u = 0;
        if (u > u_max) u = u_max;

        double current_len = get_arc_length(t, t + u);
        double error = current_len - dist;

        // If error is small enough, break
        if (std::abs(error) < 0.01) break;

        // f'(u) = speed at the current guess
        double speed = get_s_dot(t + u).norm();

        // Avoid division by zero if stopped
        if (speed < 1e-4) break; 

        // Newton step: u_new = u - f(u)/f'(u)
        u = u - (error / speed);
    }
    
    // Final clamp to ensure we stay inside the spline segment
    return std::clamp(t + u, 0.0, 1.0);
}

// Numerical integration of ||s'(t)|| from 0 to 1
void CatmulRom::calc_arc_length() {
    L_= get_arc_length(0.0, 1.0);
}

// Numerical integration of ||s'(t)|| from a to b
double CatmulRom::get_arc_length(double a, double b) {
    double length = 0.0;
    int num_samples = 10;
    double dt = (b-a) / num_samples;
    
    for (int i = 0; i < num_samples; i++) {
        double t0 = a + i * dt;
        double t1 = a + (i + 1) * dt;
        double tm = (t0 + t1) / 2.0;
        
        // Simpson's rule
        double f0 = get_s_dot(t0).norm();
        double fm = get_s_dot(tm).norm();
        double f1 = get_s_dot(t1).norm();
        
        length += (dt / 6.0) * (f0 + 4.0 * fm + f1);
    }
    
    return length;
}