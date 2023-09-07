#include "NarrowFinder.h"

vector<vector<float>> NarrowFinder::CalculatePassageValues()
{
    tic("CalculatePassageValues");
    vector<vector<Point>> components = findConnectedComponentsFree(this->Map, false, 1);
    vector<vector<Point>> componentsFilled = findConnectedComponentsFree(this->Map, false, 1, true);

    vector<Point>FreeSpace;
         
    for (int i=0; i<Map.size(); i++)
    {
        for (int j=0; j<Map[0].size(); j++)
        {
            if (Map[i][j]==0)
                FreeSpace.push_back({j, i});
        }
    }

    vector<pair<Point,Point>> ForeignMatches = ForeignMatcher(components);
    // vector<pair<Point,Point>> InvaderMatches  = InvaderOwnMatcher(components[0], FreeSpace);
    
    vector<pair<Point,Point>> SemiInvaderMatches = SemiInvaderOwnMatcher(components, componentsFilled); 

    std::vector<std::pair<Point, Point>> Matches;
    // Matches.reserve(ForeignMatches.size() + InvaderMatches.size() + SemiInvaderMatches.size());
    Matches.reserve(ForeignMatches.size() + SemiInvaderMatches.size());

    Matches.insert(Matches.end(), ForeignMatches.begin(), ForeignMatches.end());
    // Matches.insert(Matches.end(), InvaderMatches.begin(), InvaderMatches.end());
    Matches.insert(Matches.end(), SemiInvaderMatches.begin(), SemiInvaderMatches.end());

    vector<vector<float>> PassageValues = MatchesCollisionChecker(Matches);
    // for (auto& match : SemiInvaderMatches)
    // {
    //     Renderer->drawMatch(match.first, match.second);
    // }

    vector<vector<float>> PassageValuesEmpty(Map.size(), vector<float>(Map[0].size(), 500));

    toc("CalculatePassageValues");
    printAllTimes();

    Renderer->drawMatches(PassageValues);
    Renderer->Run(); 

    return PassageValues;
}

Point NarrowFinder::FindNearest(const vector<Point> &OtherUnits, Point point)
{
    tic("FindNearest");
    float minDistance = FLT_MAX;
    Point minPoint = {0, 0};

    for (int i = 0; i < OtherUnits.size(); i++)
    {
        float distance = CalculateDistance(OtherUnits[i], point);

        if (distance < minDistance)
        {
            minDistance = distance;
            minPoint = OtherUnits[i];
        }
    }

    toc("FindNearest");
    return minPoint;
}

int NarrowFinder::sumElements(const vector<vector<int>>& matrix) {
    tic("sumElements");
    int total = 0;

    for (const auto& row : matrix) {
        for (int val : row) {
            total += val;
        }
    }
    toc("sumElements");

    return total;
}

vector<pair<Point,Point>> NarrowFinder::ForeignMatcher(const vector<vector<Point>>& connectedComponents)
{
    tic("ForeignMatcher");
    int NumMatches = sumElements(Map);
    vector<pair<Point,Point>>ForeignMatches;
    ForeignMatches.reserve(NumMatches);

    if (connectedComponents.size() == 1)
        return vector<pair<Point,Point>>();

    //Her kume icin dongu
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
            if (OtherUnits.size() == 0)
                continue;
            Point p = FindNearest(OtherUnits, connectedComponents[i][j]);
            ForeignMatches.push_back({ p, connectedComponents[i][j] });
        }
    }

    toc("ForeignMatcher");
    return ForeignMatches;
}

vector<Point> NarrowFinder::bresenham(Point p0, Point p1) {
    tic("bresenham");
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

    toc("bresenham");
    return line;
}

vector<vector<float>> NarrowFinder::MatchesCollisionChecker(const vector<pair<Point,Point>> &Matches)
{
    tic("MatchesCollisionChecker");
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

    toc("MatchesCollisionChecker");
    return PassageValues;
}

vector<vector<Point>> NarrowFinder::findConnectedComponents(const vector<vector<int>> &Map, Point TopLeft, Point BotRight, int ComponentValue)
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
            if (!visited[i][j] && Map[i][j] == ComponentValue) {
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
                            !visited[newY][newX] && Map[newY][newX] == ComponentValue) {
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

vector<vector<Point>> NarrowFinder::findConnectedComponentsFree(const vector<vector<int>> &map, bool dir4, int ComponentValue, bool Filled, Point refValue)
{
    tic("findConnectedComponentsFree");
    int cols = map[0].size();
    int rows = map.size();

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Point>> components;

    vector<Point> directions;

    if (dir4)
        directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    else
        directions = { {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1} };

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (!visited[i][j] && map[i][j] == ComponentValue) {
                std::queue<Point> q;
                std::vector<Point> currentComponent;

                q.push({j, i});
                visited[i][j] = true;

                while (!q.empty()) {
                    Point current = q.front();
                    q.pop();
                    currentComponent.push_back({current.x + refValue.x, current.y + refValue.y});

                    for (Point dir : directions) {
                        int newX = current.x + dir.x;
                        int newY = current.y + dir.y;

                        if (newX >= 0 && newX < cols && newY >= 0 && newY < rows &&
                            !visited[newY][newX] && map[newY][newX] == ComponentValue) {
                            visited[newY][newX] = true;
                            q.push({newX, newY});
                        }
                    }
                }
                components.push_back(currentComponent);
            }
        }
    }

    if (!Filled)
    {
        vector<vector<Point>> newComponents(components.size());
        for (int i = 0; i < newComponents.size(); i++)
        {
            newComponents[i] = BorderPolygon(map, components[i], ComponentValue, refValue);
        }
        return newComponents;
    }

    toc("findConnectedComponentsFree");
    return components;
}

vector<Point> NarrowFinder::BorderPolygon(const vector<vector<int>> &map, vector<Point> component, int ComponentValue, Point refValue)
{
    tic("BorderPolygon");
    std::vector<int> directionsX = {0, 1, 0, -1, 1, -1, 1, -1};
    std::vector<int> directionsY = {1, 0, -1, 0, -1, 1, 1, -1};
    vector<Point> newComponent;

    for (auto& j : component)
    {
        for (size_t k = 0; k < directionsX.size(); k++)
        {
            int x = j.x + directionsX[k] - refValue.x;
            int y = j.y + directionsY[k] - refValue.y;
        
            if (y >= map.size() || x >= map[y].size() || x < 0 || y < 0)
                continue;

            if (map[y][x] == !ComponentValue)
            {
                newComponent.push_back(j);
                break;
            }
        }
    }
    toc("BorderPolygon");
    return newComponent;
}

int recursion = 0;
vector<pair<Point,Point>> NarrowFinder::InvaderOwnMatcher(const vector<Point> &connectedComponent, const vector<Point>&FreeSpace)
{
    tic("InvaderOwnMatcher");
    vector<Point> Convex = ConvexFinder::ConvexHull(FreeSpace);
    if (Convex.size() == 0)
        cout << "Convex suan 0, Freespace: " << FreeSpace.size() << endl;
    pair<vector<Point>, vector<Point>> PolygonPoints = ConvexFinder::ClusterConvexPolygon(Convex, connectedComponent);    

    vector<vector<int>> InsidePointMap(Map.size(), vector<int>(Map[0].size(),0));

    for(Point &p : PolygonPoints.second)
    {
        InsidePointMap[p.y][p.x] = 1;
    }    

    vector<vector<Point>> components_InvaderObstacle = findConnectedComponentsFree(InsidePointMap);
    vector<vector<Point>> components_InvaderObstacleFilled = findConnectedComponentsFree(InsidePointMap, false, 1, true);
    // vector<vector<float>> PassageValuesEmpty(Map.size(), vector<float>(Map[0].size(), 500));
    // Renderer->drawMatches(PassageValuesEmpty);
    
    // for(auto& p : components_InvaderObstacle)
    // {
    //     Renderer->drawPoints(p);
    // }


    // Renderer->Run();
    // while (true);
    recursion++;

    // tum ic componentleri birlestir
    std::vector<Point> insideComponents;
    size_t totalPoints = 0;
    for (const auto& innerVector : components_InvaderObstacle) {
        totalPoints += innerVector.size();
    }
    insideComponents.reserve(totalPoints);
    for (const auto& innerVector : components_InvaderObstacle) {
        insideComponents.insert(insideComponents.end(), innerVector.begin(), innerVector.end());
    }

    // for(auto& comp : components_inside_outside)
    // {
    //     comp = BorderPolygon(Map, comp, 1);
    // }

    PolygonPoints.first = BorderPolygon(Map, PolygonPoints.first, 1);

    vector<vector<Point>> components_inside_outside = {insideComponents, PolygonPoints.first};

    vector<pair<Point, Point>> outsideMatches = ForeignMatcher(components_inside_outside);
    vector<pair<Point, Point>> insideMatches = ForeignMatcher(components_InvaderObstacle);

    // if (recursion == 1)
    // {
    //     // for (auto& p : components_inside_outside)
    //     // {
    //     //     Renderer->drawPoints(p);
    //     // }
    //     // Renderer->drawPoints(PolygonPoints.first);

    //     cout <<"yafes"<<components_inside_outside.size() << endl;
    //     cout <<"enes"<<components_inside_outside[0].size()<<endl;
    //     cout <<"sahiner"<<components_inside_outside[1].size()<<endl;


    //     for (auto& match : insideMatches)
    //     {
    //         Renderer->drawMatch(match.first, match.second);
    //     }

    //     for (auto& match : outsideMatches)
    //     {
    //         Renderer->drawMatch(match.first, match.second, sf::Color::Blue);
    //     }

    //     Renderer->drawPoints(Convex);
    // }   

    // HeatMapRenderer rend(Map);
    // for (auto &i : components_InvaderObstacle)
    // {
    //     rend.drawPoints(i);
    // }
    // cout << "Convex sayisi: " << Convex.size() << endl;
    // rend.drawPoints(Convex);
    // rend.Run();
    vector<pair<Point, Point>> componenetSemiMatches = SemiInvaderOwnMatcher(components_InvaderObstacle, components_InvaderObstacleFilled);
    
    vector<pair<Point, Point>> allInvaderMatches;
    allInvaderMatches.reserve(insideMatches.size() + outsideMatches.size());
    allInvaderMatches.insert(allInvaderMatches.end(), insideMatches.begin(), insideMatches.end());
    allInvaderMatches.insert(allInvaderMatches.end(), outsideMatches.begin(), outsideMatches.end());
    allInvaderMatches.insert(allInvaderMatches.end(), componenetSemiMatches.begin(), componenetSemiMatches.end());
    
    toc("InvaderOwnMatcher");   
   return allInvaderMatches;
}

vector<pair<Point, Point>> NarrowFinder::SemiInvaderOwnMatcher(const vector<vector<Point>> &connectedComponents, const vector<vector<Point>> &connectedComponentsFilled)
{   
    tic("SemiInvaderOwnMatcher");
    vector<pair<Point, Point>> SemiInvaderMatches;

    cout << "semi girdi, recursion: " << recursion << ", component size: " << connectedComponents.size() << endl;
    for (int i=0; i<connectedComponents.size(); i++)
    {
        vector<Point> Convex = ConvexFinder::ConvexHull(connectedComponents[i]);
   
        pair<Point, Point> ConvexRect = ConvexFinder::ConvexToRect(Convex);

        // copy values in area ConvexRect to RectMap
        vector<vector<int>> RectMap(ConvexRect.second.y - ConvexRect.first.y+1, vector<int>(ConvexRect.second.x - ConvexRect.first.x+1));
        for (int i = ConvexRect.first.y; i <= ConvexRect.second.y; i++)
        {
            for (int j = ConvexRect.first.x; j <= ConvexRect.second.x; j++)
                RectMap[i-ConvexRect.first.y][j-ConvexRect.first.x] = Map[i][j];
        }
        // Convex şeklin engel olarak RectMap'e eklenmesi
        for (int i = 1; i < Convex.size(); i++)
        {
            vector<Point> points = bresenham(Convex[i-1], Convex[i]);
            for (const Point& p : points)
            {
                RectMap[p.y-ConvexRect.first.y][p.x-ConvexRect.first.x] = 1;
            }
        }
        vector<vector<Point>> ClusterFreeInPolygon = findConnectedComponentsFree(RectMap, true, 0, false, ConvexRect.first);

        // for (auto& i : ClusterFreeInPolygon)
        //     Renderer->drawPoints(i); 

        // Poligon dışındaki kümeler boş küme olarak güncellendi
        for (int j=0; j<ClusterFreeInPolygon.size(); j++)
        {    
            if (ConvexFinder::isInsideConvex(Convex, ClusterFreeInPolygon[j][0]))
            {                
                if (ClusterFreeInPolygon[j].size() < 3)
                    continue;
                vector<pair<Point, Point>> Matches = InvaderOwnMatcher(connectedComponentsFilled[i], ClusterFreeInPolygon[j]);
                SemiInvaderMatches.insert(SemiInvaderMatches.end(), Matches.begin(), Matches.end());
            }
        }              
    }
    toc("SemiInvaderOwnMatcher");
    return SemiInvaderMatches;
}

