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
typedef Kernel::Circle_2                                  Circle_2;

#include "print_utils.h"
#define PI 3.14159265

#include <ros/ros.h>

struct Coord
{
  // Position
  double x;
  double y;
};

// To check if two points are inside a rectangle
bool check_point_inside(Point_2 pt_l, Point_2 pt_r, Point_2 pt1_des, Point_2 pt2_des){
  Coord pt_l_clo, pt_l_far;
  Coord pt_r_clo, pt_r_far;
  Coord pt_left, pt_right;
  Coord des_pt_left, des_pt_right;
  double obst2boat_angle = PI/2;
  double thickness = 0.1;

  // Pasar puntos a body en implementaci[on]

  // Check which is the desired left point and which is the right
  if(pt1_des.x() < pt2_des.x() && pt1_des.y() < pt2_des.y()){
    des_pt_left.x  = to_double(pt1_des.x());
    des_pt_left.y  = to_double(pt1_des.y());
    des_pt_right.x = to_double(pt2_des.x());
    des_pt_right.y = to_double(pt2_des.y());
  } else {
    if(pt1_des.x() > pt2_des.x() && pt1_des.y() > pt2_des.y()){
      des_pt_left.x  = to_double(pt2_des.x());
      des_pt_left.y  = to_double(pt2_des.y());
      des_pt_right.x = to_double(pt1_des.x());
      des_pt_right.y = to_double(pt1_des.y());
    } else {
      if(pt1_des.x() == pt2_des.x()){
        if(pt1_des.y() < pt2_des.y()){
          des_pt_left.x  = to_double(pt1_des.x());
          des_pt_left.y  = to_double(pt1_des.y());
          des_pt_right.x = to_double(pt2_des.x());
          des_pt_right.y = to_double(pt2_des.y());
        } else {
          des_pt_left.x  = to_double(pt2_des.x());
          des_pt_left.y  = to_double(pt2_des.y());
          des_pt_right.x = to_double(pt1_des.x());
          des_pt_right.y = to_double(pt1_des.y());
        }
      } else {
        if(pt1_des.y() == pt2_des.y()){
          if(pt1_des.x() < pt2_des.x()){
            des_pt_left.x  = to_double(pt2_des.x());
            des_pt_left.y  = to_double(pt2_des.y());
            des_pt_right.x = to_double(pt1_des.x());
            des_pt_right.y = to_double(pt1_des.y());
          } else {
            des_pt_left.x  = to_double(pt1_des.x());
            des_pt_left.y  = to_double(pt1_des.y());
            des_pt_right.x = to_double(pt2_des.x());
            des_pt_right.y = to_double(pt2_des.y());
          }
        /*} else {
          ROS_ERROR("Points coincide");*/
        }
      }
    }
  }

  ROS_INFO("Left pt: (%f, %f)", des_pt_left.x, des_pt_left.y);
  ROS_INFO("Right pt: (%f, %f)", des_pt_right.x, des_pt_right.y);

  // Check which is the desired left point and which is the right
  if(pt_l.x() < pt_r.x() && pt_l.y() < pt_r.y()){
    pt_left.x  = to_double(pt_l.x());
    pt_left.y  = to_double(pt_l.y());
    pt_right.x = to_double(pt_r.x());
    pt_right.y = to_double(pt_r.y());
  } else {
    if(pt_l.x() > pt_r.x() && pt_l.y() > pt_r.y()){
      pt_left.x  = to_double(pt_r.x());
      pt_left.y  = to_double(pt_r.y());
      pt_right.x = to_double(pt_l.x());
      pt_right.y = to_double(pt_l.y());
    } else {
      if(pt_l.x() == pt_r.x()){
        if(pt_l.y() < pt_r.y()){
          pt_left.x  = to_double(pt_l.x());
          pt_left.y  = to_double(pt_l.y());
          pt_right.x = to_double(pt_r.x());
          pt_right.y = to_double(pt_r.y());
        } else {
          pt_left.x  = to_double(pt_r.x());
          pt_left.y  = to_double(pt_r.y());
          pt_right.x = to_double(pt_l.x());
          pt_right.y = to_double(pt_l.y());
        }
      } else {
        if(pt_l.y() == pt_r.y()){
          if(pt_l.x() < pt_r.x()){
          // Left el de arriba, right el de abajo
            pt_left.x  = to_double(pt_r.x());
            pt_left.y  = to_double(pt_r.y());
            pt_right.x = to_double(pt_l.x());
            pt_right.y = to_double(pt_l.y());
          } else {
          // Left el de arriba, right el de abajo
            pt_left.x  = to_double(pt_l.x());
            pt_left.y  = to_double(pt_l.y());
            pt_right.x = to_double(pt_r.x());
            pt_right.y = to_double(pt_r.y());
          }
        /*} else {
          ROS_ERROR("Points coincide");*/
        }
      }
    }
  }

  pt_l_clo.x = pt_left.x  - thickness*cos(obst2boat_angle);
  pt_l_clo.y = pt_left.y  - thickness*sin(obst2boat_angle);
  pt_r_clo.x = pt_right.x - thickness*cos(obst2boat_angle);
  pt_r_clo.y = pt_right.y - thickness*sin(obst2boat_angle);

  pt_l_far.x = pt_left.x  + thickness*cos(obst2boat_angle);
  pt_l_far.y = pt_left.y  + thickness*sin(obst2boat_angle);
  pt_r_far.x = pt_right.x + thickness*cos(obst2boat_angle);
  pt_r_far.y = pt_right.y + thickness*sin(obst2boat_angle);
  
  ROS_INFO("Left pt: (%f, %f)", pt_left.x, pt_left.y);
  ROS_INFO("Right pt: (%f, %f)", pt_right.x, pt_right.y);
  ROS_INFO("Rectangle: (%f, %f), (%f, %f), (%f, %f), (%f, %f)", pt_l_clo.x, pt_l_clo.y, pt_l_far.x, pt_l_far.y, 
                                                                pt_r_far.x, pt_r_far.y, pt_r_clo.x, pt_r_clo.y);

  if((0 <= obst2boat_angle && obst2boat_angle < PI/2) && (-PI/2 < obst2boat_angle && obst2boat_angle < -PI)){
    if((pt_l_far.y < des_pt_left.y && des_pt_left.y < pt_r_clo.y) || (pt_l_clo.y < des_pt_left.y && des_pt_left.y < pt_r_far.y)){
      if((pt_l_far.x > des_pt_left.x && des_pt_left.x > pt_r_clo.x) || (pt_l_clo.x > des_pt_left.x && des_pt_left.x > pt_r_far.x)){
        if((pt_l_far.y < des_pt_right.y && des_pt_right.y < pt_r_clo.y) || (pt_l_clo.y < des_pt_right.y && des_pt_right.y < pt_r_far.y)){
          if((pt_l_far.x > des_pt_right.x && des_pt_right.x > pt_r_clo.x) || (pt_l_clo.x > des_pt_right.x && des_pt_right.x > pt_r_far.x)){
            ROS_INFO("Desired velocities inside rectangle. Risk of collision");
            return 1;
          }
        }
      }
    }
  }
  
  if((0 < obst2boat_angle && obst2boat_angle < -PI/2) && (PI/2 < obst2boat_angle && obst2boat_angle <= PI)){
    if((pt_l_far.y < des_pt_left.y && des_pt_left.y < pt_r_clo.y) || (pt_l_clo.y < des_pt_left.y && des_pt_left.y < pt_r_far.y)){
      if((pt_l_far.x < des_pt_left.x && des_pt_left.x < pt_r_clo.x) || (pt_l_clo.x < des_pt_left.x && des_pt_left.x < pt_r_far.x)){
        if((pt_l_far.y < des_pt_right.y && des_pt_right.y < pt_r_clo.y) || (pt_l_clo.y < des_pt_right.y && des_pt_right.y < pt_r_far.y)){
          if((pt_l_far.x < des_pt_right.x && des_pt_right.x < pt_r_clo.x) || (pt_l_clo.x < des_pt_right.x && des_pt_right.x < pt_r_far.x)){
            ROS_INFO("Desired velocities inside rectangle. Risk of collision");
            return 1;
          }
        }
      }
    }
  } 

  if((obst2boat_angle == -PI/2) || (obst2boat_angle == PI/2)){
    if(pt_l_far.y > des_pt_left.y && des_pt_left.y > pt_r_clo.y){
      if(pt_l_far.x > des_pt_left.x && des_pt_left.x > pt_r_clo.x){
        if(pt_l_far.y > des_pt_right.y && des_pt_right.y > pt_r_clo.y){
          if(pt_l_far.x > des_pt_right.x && des_pt_right.x > pt_r_clo.x){
            ROS_INFO("Desired velocities inside rectangle. Risk of collision");
            return 1;
          }
        }
      }
    }
  }
  
  return 0;
}

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
  // Print example
  print_polygon (C_union.outer_boundary());
  
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
    std::cout << "-. ";
    print_polygon_with_holes (*it);
  }*/

  /*// Compute the symmetric difference of P and Q.
  Pwh_list_2 symmR;
  Pwh_list_2::const_iterator it;
  CGAL::symmetric_difference (P, Q, std::back_inserter(symmR));
  std::cout << "The symmetric difference:" << std::endl;
  for (it = symmR.begin(); it != symmR.end(); ++it) {
    std::cout << "-. ";
    print_polygon_with_holes (*it);
  }*/

  // Compute difference of two polygons
  Pwh_list_2 C_union_diff;
  Pwh_list_2::const_iterator i;

  CGAL::difference (C_union, C, std::back_inserter(C_union_diff));
  std::cout << "The difference:" << std::endl;
  for (i = C_union_diff.begin(); i != C_union_diff.end(); ++i) {
    std::cout << "-. ";
    print_polygon_with_holes (*i);
  }
  

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
    std::cout << "-. ";
    print_polygon_with_holes (*it);
  }

  // Construct the two initial polygons and the clipping rectangle.
  P.clear();
  P.push_back (Point_2 (0, 1));
  P.push_back (Point_2 (2, 0));
  P.push_back (Point_2 (1, 1));
  P.push_back (Point_2 (2, 2));
  Q.clear();
  Q.push_back (Point_2 (3, 1));
  Q.push_back (Point_2 (1, 2));
  Q.push_back (Point_2 (2, 1));
  Q.push_back (Point_2 (1, 0));
  Polygon_2 rect;
  rect.push_back (Point_2 (0, 0));
  rect.push_back (Point_2 (3, 0));
  rect.push_back (Point_2 (3, 2));
  rect.push_back (Point_2 (0, 2));
  // Perform a sequence of operations.
  S.clear();
  S.insert (P);
  S.join (Q);                   // Compute the union of P and Q.
  S.complement();               // Compute the complement.
  S.intersection (rect);        // Intersect with the clipping rectangle.
  // Print the result.

  std::cout << "The result contains " << S.number_of_polygons_with_holes()
            << " components:" << std::endl;
  S.polygons_with_holes (std::back_inserter (res));
  for (it = res.begin(); it != res.end(); ++it) {
    std::cout << "-. ";
    print_polygon_with_holes (*it);
  }

  // The polygon's outer boundary must be counter-clockwise oriented. 
  Polygon_2 d_velocity; // Line
  d_velocity.push_back(Point_2(0,0));
  d_velocity.push_back(Point_2(0.1,0));
  d_velocity.push_back(Point_2(3.1,3));
  d_velocity.push_back(Point_2(3,3));

  Polygon_set_2 ps;
  ps.insert(P);

  // Polygon::Segment_2 vel;

  if(ps.do_intersect(d_velocity)){
    std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
  } else {
    std::cout << "LOS desired vel and hdng is outside the collision cones or in boundary.\n";
  }

  Point_2 center = Point_2(0, 0);
  double radius = 2;
  Circle_2 circle(center, radius);
  
  Point_2 pt_l(-1,0),  pt_r(1,0);
  Point_2 pt1_d(-0.5,0), pt2_d(0.5,0);
  std::cout << check_point_inside(pt_l, pt_r, pt1_d, pt2_d);
  
  return 0;
}

// bool check_point_inside(Polygon_2 pol, Point_2 pt){
//     switch(CGAL::bounded_side_2(pol.vertices_begin(), pol.vertices_end(), Point_2(pt(0),pt(1)), Kernel())) {
//     case CGAL::ON_BOUNDED_SIDE:
//       std::cout << "Collision incoming.\n";
//       return 1;
//       break;
//     case CGAL::ON_BOUNDARY:
//       // std::cout << "LOS desired vel and hdng is on the collision cones boundary.\n";
//       break;
//     case CGAL::ON_UNBOUNDED_SIDE:
//       // std::cout << "LOS desired vel and hdng is outside the collision cones.\n";
//       break;
//   }
//   return 0;
// }

// bool check_vel_collision(){
//   std_msgs::Bool collision_flag;
//   collision_flag.data=0;
//   Pwh_list_2 res;
//   Pwh_list_2::const_iterator it;
//   // Polygon_2::Edge_const  _interator eit;
//   CCs_.polygons_with_holes (std::back_inserter (res));
//   Eigen::Vector3f pt = Body2NED();
//   Kernel::Segment_2 d_velocity(Point_2(pos_y_,pos_x_),Point_2(pt(1),pt(0)));
//   // std::cout<<"Vel: "<<d_velocity.source()<<" "<<d_velocity.target()<<"\n";
//   // Polygon_2 d_velocity;
//   // d_velocity.push_back(Point_2(pos_x_,pos_y_));
//   // d_velocity.push_back(Point_2(pos_x_+0.1,pos_y_));
//   // d_velocity.push_back(Point_2(pt(0)+0.1,pt(1)));
//   // d_velocity.push_back(Point_2(pt(0),pt(1)));

//   for (it = res.begin(); it != res.end(); ++it) {
//     // std::cout<<"Vel: "<<d_velocity.source()<<" "<<d_velocity.target()<<"\n";

//     // std::cout << "-. ";
//     // print_polygon_with_holes (*it);
//     // std::cout << "Pt vel: " << pt << "\n";

//     // std::cout << CGAL::is_simple_2((*it).outer_boundary().vertices_begin(), (*it).outer_boundary().vertices_end(), Kernel()) << "\n";

//     // CGAL uses same coordinate system as RVIZ? front - x left - y
//     // NEgar y cuando se trabaje con rviz
//     // if (CGAL::bounded_side_2((*it).outer_boundary().vertices_begin(), (*it).outer_boundary().vertices_end(), Point_2(pt(0),pt(1)), Kernel())
//     //     == CGAL::ON_BOUNDED_SIDE){
//     for (eit = (*it).outer_boundary().edges_begin(); eit != (*it).outer_boundary().edges_end(); ++eit){
//       if (CGAL::do_intersect(d_velocity, *eit)){
//         // std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
//         collision_flag.data=1;
//         collision_pub_.publish(collision_flag);
//         return 1;
//       } else {
//       // std::cout << "LOS desired vel and hdng is outside the collision cones or in boundary.\n";
//         collision_pub_.publish(collision_flag);
//       }
//     }
//      // if(CGAL::do_intersect((*it).outer_boundary(),d_velocity)){
//       //   std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
//       //   collision_flag.data=1;
//       //   collision_pub_.publish(collision_flag);
//       //   return 1;
//       // } else {
//       //   std::cout << "LOS desired vel and hdng is outside the collision cones or in boundary.\n";
//       //   collision_pub_.publish(collision_flag);
//       // }
//   }
//   return 0;

//   //   switch(CGAL::bounded_side_2((*it).outer_boundary().vertices_begin(), (*it).outer_boundary().vertices_end(), Point_2(pt(0),pt(1)), Kernel())) {
//   //     case CGAL::ON_BOUNDED_SIDE:
//   //       std::cout << "LOS desired vel and hdng is inside the collision cones.\n";
//   //       collision_pub_.publish(collision_flag);
//   //       return 1;
//   //       break;
//   //     case CGAL::ON_BOUNDARY:
//   //       std::cout << "LOS desired vel and hdng is on the collision cones boundary.\n";
//   //       break;
//   //     case CGAL::ON_UNBOUNDED_SIDE:
//   //       std::cout << "LOS desired vel and hdng is outside the collision cones.\n";
//   //       break;
//   //   }
//   // }
//   // return 0;
// }

