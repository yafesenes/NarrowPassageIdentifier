#ifndef POINT_CLASS_H
#define POINT_CLASS_H

struct Point {
    int x, y;
    Point() : x(0), y(0) {}
    Point(int x, int y) : x(x), y(y) {}
};

#endif