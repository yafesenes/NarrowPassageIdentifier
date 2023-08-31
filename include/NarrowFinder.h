#ifndef NARROWFINDER_CLASS_H
#define NARROWFINDER_CLASS_H

#include <vector>
#include <cmath>
#include "Point.h"
#include <queue>
#include <float.h>
#include "Convex.h"

using namespace std;

class NarrowFinder{
private:
    vector<vector<int>> Map;

private:
    float CalculateDistance(Point p0, Point p1)
    {
        return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
    }

    vector<vector<Point>> findConnectedComponents(const vector<vector<int>> &Map, Point TopLeft = {0, 0}, Point BotRight = {-1, -1});
    Point FindNearest(const vector<Point> &OtherUnits, Point point);
    int sumElements(const vector<vector<int>>& matrix);
    vector<pair<Point,Point>> ForeignMatcher(const vector<vector<Point>>& connectedComponents);    
    vector<Point> bresenham(Point p0, Point p1);
    vector<vector<float>> MatchesCollisionChecker(const vector<pair<Point,Point>> &Matches);
    vector<pair<Point,Point>> InvaderOwnMatcher(const vector<vector<Point>> &connectedComponents);
    // vector<pair<Point, Point>> SemiInvaderOwnMatcher(const vector<vector<Point>> &connectedComponents);

public:
    NarrowFinder(vector<vector<int>> &Map)
    {
        this->Map = Map;        
    }

    vector<vector<float>> CalculatePassageValues();

    

};

#endif