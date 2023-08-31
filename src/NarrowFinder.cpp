#include "NarrowFinder.h"
#include "Convex.h"
#include "Renderer.h"
#include <thread>

vector<vector<float>> NarrowFinder::CalculatePassageValues()
{
    vector<vector<Point>> components = findConnectedComponents(this->Map);
    vector<pair<Point,Point>> ForeignMatches = ForeignMatcher(components);
    vector<pair<Point,Point>> InvaderMatches  = InvaderOwnMatcher(components);

    std::vector<std::pair<Point, Point>> Matches;
    Matches.reserve(ForeignMatches.size() + InvaderMatches.size());

    Matches.insert(Matches.end(), ForeignMatches.begin(), ForeignMatches.end());
    Matches.insert(Matches.end(), InvaderMatches.begin(), InvaderMatches.end());

    vector<vector<float>> PassageValues = MatchesCollisionChecker(Matches);

    HeatMapRenderer renderer(PassageValues, Map);
    // for (auto& match : Matches)
    // {
    //     renderer.drawMatch(match.first, match.second);
    // }
    renderer.Run(); 
   
    // MapRenderer renderer(Map);
    // renderer.Run();

    return PassageValues;
}

Point NarrowFinder::FindNearest(const vector<Point> &OtherUnits, Point point)
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

int NarrowFinder::sumElements(const vector<vector<int>>& matrix) {
    int total = 0;

    for (const auto& row : matrix) {
        for (int val : row) {
            total += val;
        }
    }

    return total;
}

vector<pair<Point,Point>> NarrowFinder::ForeignMatcher(const vector<vector<Point>>& connectedComponents)
{
    int NumMatches = sumElements(Map);
    vector<pair<Point,Point>>ForeignMatches;
    ForeignMatches.reserve(NumMatches);

    if (connectedComponents.size() == 1)
        return vector<pair<Point,Point>>();

    //Her k�me i�in d�ng�
    for (int i = 0; i < connectedComponents.size(); i++)
    {
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

vector<Point> NarrowFinder::bresenham(Point p0, Point p1) {
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

vector<vector<float>> NarrowFinder::MatchesCollisionChecker(const vector<pair<Point,Point>> &Matches)
{
    vector<vector<float>> PassageValues(Map.size(), vector<float>(Map[0].size(),min(Map.size(),Map[0].size())));

    if (Matches.size() == 0)
        return PassageValues;

    for (int i=0; i < Matches.size(); i++)
    {
        bool flag = 0;
        float distance = CalculateDistance(Matches[i].first, Matches[i].second);

        vector<Point> midPoints = bresenham(Matches[i].first, Matches[i].second);

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

vector<vector<Point>> NarrowFinder::findConnectedComponents(const vector<vector<int>> &Map, Point TopLeft, Point BotRight)
{
    int rows = Map.size();
    int cols = Map[0].size();

    if (BotRight.x == -1)
        BotRight = {rows, cols};

    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    vector<vector<Point>> components;

    vector<Point> directions = { {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1} };

    for (int i = TopLeft.x; i < BotRight.x; ++i) {
        for (int j = TopLeft.y; j < BotRight.y; ++j) {
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

vector<pair<Point,Point>> NarrowFinder::InvaderOwnMatcher(const vector<vector<Point>> &connectedComponents)
{
    vector<Point>FreeSpace;
         
    for (int i=0; i<Map.size(); i++)
    {
        for (int j=0; j<Map[0].size(); j++)
        {
            if (Map[i][j]==0)
                FreeSpace.push_back({i,j});
        }
    }

    vector<Point> Convex = ConvexFinder::ConvexHull(FreeSpace);
    pair<vector<Point>, vector<Point>> PolygonPoints = ConvexFinder::ClusterConvexPolygon(Convex, connectedComponents[0]);    

    vector<vector<int>> InsidePointMap(Map.size(), vector<int>(Map[0].size(),0));

    for(Point &p : PolygonPoints.second)
    {
        InsidePointMap[p.y][p.x] = 1;
    }    

    vector<vector<Point>> components_InvaderObstacle = findConnectedComponents(InsidePointMap);

    components_InvaderObstacle.push_back(PolygonPoints.first);


    return ForeignMatcher(components_InvaderObstacle);     
}

// vector<pair<Point, Point>> NarrowFinder::SemiInvaderOwnMatcher(const vector<vector<Point>> &connectedComponents)
// {
//     // FreeSpace'i daha öncede hesaplamıştık, burası düzenlenebilir
//     vector<Point>FreeSpace;
        
//     for (int i=0; i<Map.size(); i++)
//     {
//         for (int j=0; j<Map[0].size(); j++)
//         {
//             if (Map[i][j]==0)
//                 FreeSpace.push_back({i,j});
//         }
//     }   
    
//     for (int i=1; i<connectedComponents.size(); i++)
//     {
//         vector<Point> Convex = ConvexFinder::ConvexHull(connectedComponents[i]);
//         pair<Point, Point> ConvexRect = ConvexFinder::ConvexToRect(Convex);

      
//     }    
// }

