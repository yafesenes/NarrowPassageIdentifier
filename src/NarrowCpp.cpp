#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <float.h>
#include "Renderer.h"
#include "pgm_reader.h"
#include "Image.h"

using namespace std;

struct Point {
    int x, y;
};

float CalculateDistance(Point p0, Point p1)
{
    return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
}

void PrintComponents(const vector<vector<int>>& Array)
{
    for (int i = 0; i < Array.size(); i++)
    {
        for (int j = 0; j < Array[0].size(); j++)
        {
            cout << Array[i][j] << " ";
        }

        cout << endl;
    }
}

void PrintComponents(const vector<vector<float>>& Array)
{
    for (int i = 0; i < Array.size(); i++)
    {
        for (int j = 0; j < Array[0].size(); j++)
        {
            cout << Array[i][j] << " ";
        }

        cout << endl;
    }
}

void PrintComponents(const vector<vector<Point>>& components)
{
    for (const auto& component : components) {
        for (const auto& point : component) {
            cout << "(" << point.x << ", " << point.y << "), ";
        }
        cout << endl;
    }
}

vector<vector<int>> loadMap() 
{
    Image Img("res/imgs/10.png");
    return Img.getData();
}

vector<vector<Point>> findConnectedComponents(const vector<vector<int>>& Map) {
    int rows = Map.size();
    int cols = Map[0].size();

    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    vector<vector<Point>> components;

    // 8 y�n� temsil eden vekt�rleri ekleyelim: Sa�, alt, sol, �st, sa�-alt, sol-alt, sol-�st, sa�-�st
    vector<Point> directions = { {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1} };

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (!visited[i][j] && Map[i][j] == 1) {
                queue<Point> q;
                vector<Point> currentComponent;

                q.push({ j, i });
                visited[i][j] = true;

                while (!q.empty()) {
                    Point current = q.front();
                    q.pop();
                    currentComponent.push_back(current);

                    for (Point dir : directions) {
                        int newX = current.x + dir.x;
                        int newY = current.y + dir.y;

                        if (newX >= 0 && newX < cols && newY >= 0 && newY < rows &&
                            !visited[newY][newX] && Map[newY][newX] == 1) {
                            visited[newY][newX] = true;
                            q.push({ newX, newY });
                        }
                    }
                }

                components.push_back(currentComponent);
            }
        }
    }

    std::vector<int> directionsX = {0, 1, 0, -1, 1, -1, 1, -1};
    std::vector<int> directionsY = {1, 0, -1, 0, -1, 1, 1, -1};
    std::vector<std::vector<Point>> newComponents(components.size());
    for (size_t i = 0; i < components.size(); i++)
    {
        for (auto& j : components[i])
        {
            for (size_t k = 0; k < directionsX.size(); k++)
            {
                int x = j.x + directionsX[k];
                int y = j.y + directionsY[k];
               
                if (x >= Map.size() || y >= Map[x].size())
                    continue;

                if (Map[y][x] == 0)
                {
                    newComponents[i].push_back(j);
                    break;
                }
            }
        }
    }

    return newComponents;
}

Point FindNearest(const vector<Point> &OtherUnits, Point point)
{
    double minDistance = DBL_MAX;
    Point minPoint = {0, 0};

    for (int i = 0; i < OtherUnits.size(); i++)
    {
        double distance = CalculateDistance(OtherUnits[i], point);

        if (distance < minDistance)
        {
            minDistance = distance;
            minPoint = OtherUnits[i];
        }
    }

    return minPoint;
}

int sumElements(const vector<vector<int>>& matrix) {
    int total = 0;

    for (const auto& row : matrix) {
        for (int val : row) {
            total += val;
        }
    }

    return total;
}

vector<vector<Point>> ForeignMatcher(const vector<vector<Point>>& connectedComponents, const vector<vector<int>>& Map)
{
    int NumMatches = sumElements(Map);
    vector<vector<Point>>ForeignMatches;
    ForeignMatches.reserve(NumMatches);

    if (connectedComponents.size() == 1)
        return vector<vector<Point>>();

    //Her k�me i�in d�ng�
    for (int i = 0; i < connectedComponents.size(); i++)
    {
        vector<vector<Point>> Coordinates = connectedComponents;

        Coordinates[i] = vector<Point>();
        vector<Point>OtherUnits;

        //K�me d���nda kalan di�er koordinatlar�n birle�tirilmesi
        for (int j = 0; j < connectedComponents.size(); j++)
        {
            if (i == j)
                continue;

            for (int k = 0; k < connectedComponents[j].size(); k++)
            {
                OtherUnits.push_back(connectedComponents[j][k]);
            }
        }

        //K�medeki her noktan�n e�le�mesinin teker teker hesaplanmas�
        for (int j = 0; j < connectedComponents[i].size(); j++)
        {
            Point p = FindNearest(OtherUnits, connectedComponents[i][j]);
            ForeignMatches.push_back({ p, connectedComponents[i][j] });
        }
    }
    return ForeignMatches;
}

vector<Point> bresenham(Point p0, Point p1) {
    std::vector<Point> line;

    int dx = std::abs(p1.x - p0.x);
    int dy = std::abs(p1.y - p0.y);

    int sx = (p0.x < p1.x) ? 1 : -1;
    int sy = (p0.y < p1.y) ? 1 : -1;

    int err = dx - dy;

    while (true) {
        if (p0.x == p1.x && p0.y == p1.y)
            break;

        int e2 = 2 * err;

        if (e2 > -dy) {
            err -= dy;
            p0.x += sx;
        }

        if (e2 < dx) {
            err += dx;
            p0.y += sy;
        }

        if (!(p0.x == p1.x && p0.y == p1.y))
            line.push_back(p0);
    }

    return line;
}


vector<vector<float>> MatchesCollisionChecker(const vector<vector<Point>> &Matches, const vector<vector<int>> &Map)
{
    vector<vector<float>> PassageValues(Map.size(), vector<float>(Map[0].size(),min(Map.size(),Map[0].size())));

    if (Matches.size() == 0)
        return PassageValues;

    for (int i=0; i < Matches.size(); i++)
    {
        bool flag = 0;
        float distance = CalculateDistance(Matches[i][0], Matches[i][1]);

        vector<Point> midPoints = bresenham(Matches[i][0], Matches[i][1]);

        for (int j = 0; j < midPoints.size(); j++)
        {
            if (Map[midPoints[j].y][midPoints[j].x] == 1)
            {
                flag = 1;
                break;
            } 
        }

        if (!flag) 
        {
            for (int j = 0; j < midPoints.size(); j++)
            {
                if (PassageValues[midPoints[j].y][midPoints[j].x] > distance)
                {
                    PassageValues[midPoints[j].y][midPoints[j].x] = distance;
                }
            }
        }
    }

    return PassageValues;
}

int main() {
    vector<vector<int>> Map = loadMap();

    vector<vector<Point>> components = findConnectedComponents(Map);
    vector<vector<Point>> ForeignMatches = ForeignMatcher(components, Map);
    vector<vector<float>> PassageValues = MatchesCollisionChecker(ForeignMatches, Map);
    MapRenderer renderer(Map);

    renderer.Run();

    return 0;
}
