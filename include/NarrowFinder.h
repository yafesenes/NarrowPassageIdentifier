#ifndef NARROWFINDER_CLASS_H
#define NARROWFINDER_CLASS_H

#include <vector>
#include <cmath>
#include "Point.h"
#include <queue>
#include <float.h>
#include <memory>
#include "Convex.h"
#include "Renderer.h"

using namespace std;

class NarrowFinder{
private:
    vector<vector<int>> Map;
    unique_ptr<HeatMapRenderer> Renderer;

private:
    float CalculateDistance(Point p0, Point p1)
    {
        return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
    }

    vector<vector<Point>> findConnectedComponents(const vector<vector<int>> &Map, Point TopLeft = {0, 0}, Point BotRight = {-1, -1}, int ComponentValue = 1);
    vector<vector<Point>> findConnectedComponentsFree(const vector<vector<int>> &map, bool dir4 = false, int ComponentValue = 1, bool Filled = false, Point refValue = {0, 0});
    Point FindNearest(const vector<Point> &OtherUnits, Point point);
    int sumElements(const vector<vector<int>>& matrix);
    vector<pair<Point,Point>> ForeignMatcher(const vector<vector<Point>>& connectedComponents);    
    vector<Point> bresenham(Point p0, Point p1);
    vector<vector<float>> MatchesCollisionChecker(const vector<pair<Point,Point>> &Matches);
    vector<pair<Point,Point>> InvaderOwnMatcher(const vector<Point> &connectedComponent, const vector<Point>&FreeSpace);
    vector<pair<Point, Point>> SemiInvaderOwnMatcher(const vector<vector<Point>> &connectedComponents, const vector<vector<Point>> &connectedComponentsFilled);
    vector<Point> BorderPolygon(const vector<vector<int>> &map, vector<Point> component, int ComponentValue, Point refValue = {0, 0});

public:
    NarrowFinder(vector<vector<int>> &Map)
    {
        this->Map = Map;
        Renderer = make_unique<HeatMapRenderer>(HeatMapRenderer(Map));      
    }

    vector<vector<float>> CalculatePassageValues();

    

};

#endif