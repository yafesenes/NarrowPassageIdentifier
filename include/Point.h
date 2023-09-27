#ifndef POINT_CLASS_H
#define POINT_CLASS_H

//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/point_xy.hpp>
//#include <boost/geometry/geometries/polygon.hpp>

struct Point {
    int x, y;
    Point() : x(0), y(0) {}
    Point(int x, int y) : x(x), y(y) {}
};
//typedef boost::geometry::model::d2::point_xy<int> Point;

#endif