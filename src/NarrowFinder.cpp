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

    tic("YalnizForeign");
    vector<pair<Point,Point>> ForeignMatches = ForeignMatcher(components);
    toc("YalnizForeign");
    // vector<pair<Point,Point>> InvaderMatches  = InvaderOwnMatcher(components[0], FreeSpace);
    
    tic("YalnizSemi");
    vector<pair<Point,Point>> SemiInvaderMatches = SemiInvaderOwnMatcher(components, componentsFilled);
    toc("YalnizSemi");

    std::vector<std::pair<Point, Point>> Matches;
    // Matches.reserve(ForeignMatches.size() + InvaderMatches.size() + SemiInvaderMatches.size());
    Matches.reserve(ForeignMatches.size() + SemiInvaderMatches.size());

    Matches.insert(Matches.end(), ForeignMatches.begin(), ForeignMatches.end());
    // Matches.insert(Matches.end(), InvaderMatches.begin(), InvaderMatches.end());
    Matches.insert(Matches.end(), SemiInvaderMatches.begin(), SemiInvaderMatches.end());


    vector<vector<float>> PassageValues = MatchesCollisionChecker(Matches);
    
    // for (auto& match : Matches)
    // {
    //     Renderer->drawMatch(match.first, match.second);
    // }

    // for (auto& match : Matches)
    // {
    //     vector<Point> points = {match.first, match.second};
    //     Renderer->drawPoints(points);
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

    // dikdortgenleri hesapla
    vector<pair<Point, Point>> rects;
    rects.reserve(connectedComponents.size());
    for (auto& component : connectedComponents)
    {
        if (component.size() > 0)
        {
            auto rect = ConvexFinder::ConvexToRect(component);
            rects.push_back(rect);
        }
    }

    int NumMatches = 0;
    for (auto& i : connectedComponents)
        NumMatches += i.size();

    vector<pair<Point,Point>>ForeignMatches;
    ForeignMatches.reserve(NumMatches);

    if (connectedComponents.size() == 1)
    {
        toc("ForeignMatcher");
        return vector<pair<Point,Point>>();
    }

    RTree rtree = calculateRTree(rects);

    //Her kume icin dongu
    for (int i = 0; i < connectedComponents.size(); i++)
    {        
        // vector<unsigned>OtherUnitsIndexesEski = getOtherUnitsEski(i, rects);
        // int numOtherUnits = 0;
        // for (auto& i : OtherUnitsIndexesEski)
        //     numOtherUnits += connectedComponents[i].size();
        
        // vector<Point> OtherUnitsEski;
        // OtherUnitsEski.reserve(numOtherUnits);
        // for (auto& i : OtherUnitsIndexesEski)
        //     for (auto& j : connectedComponents[i])
        //         OtherUnitsEski.push_back(j);

        //K�medeki her noktan�n e�le�mesinin teker teker hesaplanmas�
        for (int j = 0; j < connectedComponents[i].size(); j++)
        {
            vector<unsigned> OtherUnitsIndexes = getOtherUnits(rtree, connectedComponents[i][j], i);
            int numOtherUnits = 0;
            for (auto& i : OtherUnitsIndexes)
                numOtherUnits += connectedComponents[i].size();
            

            Point p11 = connectedComponents[i][j];
            pair<Point, Point> rectzz = {Point(p11.x - _thresholdValue,
                                                p11.y - _thresholdValue),
                                    Point(p11.x + _thresholdValue,
                                                p11.y + _thresholdValue)};
            vector<Point> OtherUnits;
            OtherUnits.reserve(numOtherUnits);
            for (auto& i : OtherUnitsIndexes)
                for (auto& j : connectedComponents[i])
                    if (j.x > rectzz.first.x && j.x < rectzz.second.x && j.y > rectzz.first.y && j.y < rectzz.second.y)
                        OtherUnits.push_back(j);

            if (OtherUnits.size() == 0)
                continue;
            Point p = FindNearest(OtherUnits, connectedComponents[i][j]);
            ForeignMatches.push_back({ p, connectedComponents[i][j] });
        }
    }

    toc("ForeignMatcher");
    return ForeignMatches;
}

RTree NarrowFinder::calculateRTree(const vector<pair<Point, Point>>& rects)
{
    tic("calculateRTree");
    RTree rtree;
    int i = 0;
    for (auto& rect : rects)
    {
        Box rectBox(RPoint(rect.first.x, rect.first.y), 
                    RPoint(rect.second.x, rect.second.y));
        rtree.insert(make_pair(rectBox, i++));
    }

    toc("calculateRTree");
    return rtree;
}

vector<unsigned> NarrowFinder::getOtherUnits(const RTree& rtree, Point p, size_t componentIndex)
{
    tic("getOtherUnits");
    Box queryBox(RPoint(p.x - _thresholdValue,
                        p.y - _thresholdValue),
                 RPoint(p.x + _thresholdValue,
                        p.y + _thresholdValue));
    std::vector<Value> result;
    rtree.query(bgi::intersects(queryBox), std::back_inserter(result));

    if (result.size() == 1)
    {
        toc("getOtherUnits");
        return vector<unsigned>();
    }

    vector<unsigned> indexes(result.size()-1);
    size_t j = 0;
    for (int i = 0; i < result.size(); i++)
    {
        if (result[i].second == componentIndex)
            continue;

        // cout << "j: " << j << " result: " << i.second << " comp_index: " << componentIndex << " result_size: " << result.size() << endl;
        indexes[j++] = result[i].second;
    }

    toc("getOtherUnits");
    return indexes;
}

vector<unsigned> NarrowFinder::getOtherUnitsEski(size_t componentIndex, const vector<pair<Point, Point>>& rects)
{
    tic("getOtherUnits");
    // RTree rtree;
    // int i = 0;
    // for (int j = 0; j < rects.size(); j++)
    // {
    //     if (j == componentIndex)
    //         continue;
    //     Box rectBox(RPoint(rects[j].first.x, rects[j].first.y), 
    //                 RPoint(rects[j].second.x, rects[j].second.y));
    //     rtree.insert(make_pair(rectBox, i++));
    // }
    // pair<Point, Point> rect = rects[componentIndex];
    // Box queryBox(RPoint(rect.first.x - _thresholdValue,
    //                     rect.first.y - _thresholdValue),
    //              RPoint(rect.second.x + _thresholdValue,
    //                     rect.second.y + _thresholdValue));
    
    // std::vector<Value> result;
    // rtree.query(bgi::intersects(queryBox), std::back_inserter(result));
    
    // vector<unsigned> indexes(result.size());
    // for (int i = 0; i < result.size(); i++)
    // {
    //     indexes[i] = result[i].second;
    // }

    vector<unsigned> indexes;
    for (int i = 0; i < rects.size(); i++)
        if (i != componentIndex)
            indexes.push_back(i);

    toc("getOtherUnits");
    return indexes;
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
    {
        toc("MatchesCollisionChecker");
        return PassageValues;
    }

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

        toc("findConnectedComponentsFree");
        return newComponents;
    }

    toc("findConnectedComponentsFree");
    return components;
}

vector<Point> NarrowFinder::BorderPolygon(const vector<vector<int>> &map, const vector<Point>& component, int ComponentValue, Point refValue)
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
    tic("invader1");
    vector<Point> Convex = ConvexFinder::ConvexHull(FreeSpace);
    if (Convex.size() == 0)
        cout << "Convex suan 0, Freespace: " << FreeSpace.size() << endl;
    pair<vector<Point>, vector<Point>> PolygonPoints = ConvexFinder::ClusterConvexPolygon(Convex, connectedComponent);    
    toc("invader1");

    vector<vector<int>> InsidePointMap(Map.size(), vector<int>(Map[0].size(),0));

    tic("invader111");
    for(Point &p : PolygonPoints.second)
    {
        InsidePointMap[p.y][p.x] = 1;
    }
    toc("invader111");

    tic("invader2");
    vector<vector<Point>> components_InvaderObstacleFilled = findConnectedComponentsFree(InsidePointMap, false, 1, true);
    vector<vector<Point>> components_InvaderObstacle(components_InvaderObstacleFilled.size());
    for (int i = 0; i < components_InvaderObstacle.size(); i++)
    {
        components_InvaderObstacle[i] = BorderPolygon(InsidePointMap, components_InvaderObstacleFilled[i], 1, {0, 0});
    }
    toc("invader2");

    // tum ic componentleri birlestir
    tic("invader3");
    std::vector<Point> insideComponents;
    size_t totalPoints = 0;
    for (const auto& innerVector : components_InvaderObstacle) {
        totalPoints += innerVector.size();
    }
    insideComponents.reserve(totalPoints);
    for (const auto& innerVector : components_InvaderObstacle) {
        insideComponents.insert(insideComponents.end(), innerVector.begin(), innerVector.end());
    }
    toc("invader3");

    tic("invader555");
    PolygonPoints.first = BorderPolygon(Map, PolygonPoints.first, 1);
    toc("invader555");

    tic("invader4");
    vector<vector<Point>> components_inside_outside = {insideComponents, PolygonPoints.first};

    vector<pair<Point, Point>> outsideMatches = ForeignMatcher(components_inside_outside);
    vector<pair<Point, Point>> insideMatches = ForeignMatcher(components_InvaderObstacle);

    vector<pair<Point, Point>> allInvaderMatches;
    allInvaderMatches.reserve(insideMatches.size() + outsideMatches.size());
    toc("invader4");
    recursion++;
    if (recursion < 1)
    {
        vector<pair<Point, Point>> componenetSemiMatches = SemiInvaderOwnMatcher(components_InvaderObstacle, components_InvaderObstacleFilled);
        allInvaderMatches.insert(allInvaderMatches.end(), componenetSemiMatches.begin(), componenetSemiMatches.end());
    }
    tic("invader5");
    allInvaderMatches.insert(allInvaderMatches.end(), insideMatches.begin(), insideMatches.end());
    allInvaderMatches.insert(allInvaderMatches.end(), outsideMatches.begin(), outsideMatches.end());
    toc("invader5");
    toc("InvaderOwnMatcher");   
   return allInvaderMatches;
}

vector<pair<Point, Point>> NarrowFinder::SemiInvaderOwnMatcher(const vector<vector<Point>> &connectedComponents, const vector<vector<Point>> &connectedComponentsFilled)
{   
    tic("SemiInvaderOwnMatcher");
    vector<pair<Point, Point>> SemiInvaderMatches;

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

