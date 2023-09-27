#ifndef NARROWFINDER_CLASS_H
#define NARROWFINDER_CLASS_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <queue>
#include <float.h>
#include <memory>
#include "Point.h"
#include "Convex.h"
#include "Renderer.h"
#include "TicToc.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<int, 2, bg::cs::cartesian> RPoint;
typedef bg::model::box<RPoint> Box;
typedef std::pair<Box, unsigned> Value;
typedef bgi::rtree< Value, bgi::quadratic<16> > RTree;

using namespace std;

class NarrowFinder{
private:
    vector<vector<int>> Map;
    unique_ptr<HeatMapRenderer> Renderer;
    const float _thresholdValue;

private:
    float CalculateDistance(Point p0, Point p1)
    {
        return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
    }

    vector<vector<Point>> findConnectedComponentsFree(const vector<vector<int>> &map, bool dir4 = false, int ComponentValue = 1, bool Filled = false, Point refValue = {0, 0});
    Point FindNearest(const vector<Point>& OtherUnits, Point point);
    vector<pair<Point,Point>> ForeignMatcher(const vector<vector<Point>>& connectedComponents);
    RTree calculateRTree(const vector<pair<Point, Point>>& rects);
    vector<unsigned> getOtherUnits(const RTree& rtree, Point p, size_t componentIndex);
    vector<Point> bresenham(Point p0, Point p1);
    vector<vector<float>> MatchesCollisionChecker(const vector<pair<Point,Point>> &Matches);
    vector<pair<Point,Point>> InvaderOwnMatcher(const vector<Point> &connectedComponent, const vector<Point>& FreeSpace);
    vector<pair<Point, Point>> SemiInvaderOwnMatcher(const vector<vector<Point>> &connectedComponents, const vector<vector<Point>> &connectedComponentsFilled);
    vector<Point> BorderPolygon(const vector<vector<int>> &map, const vector<Point>& component, int ComponentValue, Point refValue = {0, 0});

public:
    NarrowFinder(vector<vector<int>> &Map, float thresholdValue) :
        _thresholdValue(thresholdValue)
    {
        this->Map = Map;
        Renderer = make_unique<HeatMapRenderer>(HeatMapRenderer(Map));      
    }

    vector<vector<float>> CalculatePassageValues();

    

};

#endif