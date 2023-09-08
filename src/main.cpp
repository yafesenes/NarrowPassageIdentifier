#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <float.h>
#include "Renderer.h"
#include "pgm_reader.h"
#include "Image.h"
#include "Point.h"
#include "NarrowFinder.h"
#include "Convex.h"
#include <thread>

using namespace std;

void PrintComponents(const vector<Point>& p)
{
    for (const Point &a:p)
    {
        cout<<a.x<<" "<<a.y<<endl;
    }
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
    Image Img("res/Map2/willow.png");
    vector<vector<int>> map = Img.getData();
    vector<vector<int>> newMap = map;

    size_t rows = map.size();
    size_t cols = map[0].size();
    int inflation_radius = 2;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (map[i][j] == 1) { // Obstacle detected
                for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        int new_x = i + dx;
                        int new_y = j + dy;
                        if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols) {
                            newMap[new_x][new_y] = 1; // Inflate the obstacle
                        }
                    }
                }
            }
        }
    }

    return newMap;

    // return pgmreader::readMap("res/RealMaps/","willow.yaml");
}

int main() {
    vector<vector<int>> Map = loadMap();
    MapRenderer rend(Map);
    rend.Run();

    // for (int i = 390; i < Map.size(); i++)
    // {
    //     for (int j = 950; j < Map[0].size(); j++)
    //     {
    //         cout << Map[i][j] << " ";
    //     }
    //     cout << endl;
    //     cout << endl;
    // }
    
    NarrowFinder narrowFinder(Map);
    vector<vector<float>> passageValues = narrowFinder.CalculatePassageValues();
    
    // HeatMapRenderer renderer(passageValues, Map);
    // thread HeatMapThread(&::HeatMapRenderer::Run, &renderer);
    
    // HeatMapThread.join();
    return 0;
}
