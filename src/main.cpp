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
    Image Img("res/Map2/2.png");
    return Img.getData();
}

int main() {
    vector<vector<int>> Map = loadMap();
    
    NarrowFinder narrowFinder(Map);
    vector<vector<float>> passageValues = narrowFinder.CalculatePassageValues();
    
    // HeatMapRenderer renderer(passageValues, Map);
    // thread HeatMapThread(&::HeatMapRenderer::Run, &renderer);
    
    // HeatMapThread.join();
    return 0;
}
