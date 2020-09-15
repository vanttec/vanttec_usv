#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>
#include <list>
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;
typedef CGAL::Polygon_set_2<Kernel>                       Polygon_set_2;
#include "print_utils.h"

#include <ros/ros.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "cgal");
  ros::NodeHandle vo_node("cgal");
  ros::Rate loop_rate(100);
  // Construct the two input polygons.
  Polygon_2 C;
  C.push_back (Point_2 (0, 0));
  C.push_back (Point_2 (5, 0));
  C.push_back (Point_2 (5, 5));
  std::cout << "C = "; print_polygon (C);
  Polygon_2 P;
  P.push_back (Point_2 (4, 0));
  P.push_back (Point_2 (5, 0));
  P.push_back (Point_2 (5, 8));
  P.push_back (Point_2 (4, 8));
  std::cout << "P = "; print_polygon (P);
  Polygon_2 Q;
  Q.push_back (Point_2 (3, 3));
  Q.push_back (Point_2 (6, 3));
  Q.push_back (Point_2 (6, 6));
  Q.push_back (Point_2 (3, 6));
  std::cout << "Q = "; print_polygon (Q);
  
  // Compute the union of P and Q.
  Polygon_with_holes_2 C_union(P);
  
  if ((CGAL::do_intersect (C_union, Q)))
    std::cout << "The two polygons intersect." << std::endl;
  else
    std::cout << "The two polygons do not intersect." << std::endl;


  if (CGAL::join (C, C_union, C_union)) {
    std::cout << "The union: ";
    print_polygon_with_holes (C_union);
  } else
    std::cout << "P and Q are disjoint and their union is trivial."
              << std::endl;
  std::cout << std::endl;
    
  /*// Compute the intersection of P and Q.
  Pwh_list_2                  intR;
  Pwh_list_2::const_iterator  it;
  CGAL::intersection (P, Q, std::back_inserter(intR));
  std::cout << "The intersection:" << std::endl;
  for (it = intR.begin(); it != intR.end(); ++it) {
    std::cout << "--> ";
    print_polygon_with_holes (*it);
  }*/

  /*// Compute the symmetric difference of P and Q.
  Pwh_list_2 symmR;
  Pwh_list_2::const_iterator it;
  CGAL::symmetric_difference (P, Q, std::back_inserter(symmR));
  std::cout << "The symmetric difference:" << std::endl;
  for (it = symmR.begin(); it != symmR.end(); ++it) {
    std::cout << "--> ";
    print_polygon_with_holes (*it);
  }*/

  // Perform a sequence of operations.
  Polygon_set_2 S;
  S.insert (C_union);
  S.complement();               // Compute the complement.
  S.intersection (Q);        // Intersect with the clipping rectangle.
  // Print the result.
  std::list<Polygon_with_holes_2> res;
  std::list<Polygon_with_holes_2>::const_iterator it;
  std::cout << "The result contains " << S.number_of_polygons_with_holes()
            << " components:" << std::endl;
  S.polygons_with_holes (std::back_inserter (res));
  for (it = res.begin(); it != res.end(); ++it) {
    std::cout << "--> ";
    print_polygon_with_holes (*it);
  }

  return 0;
}
