#ifndef RENDERER_CLASS_H
#define RENDERER_CLASS_H
#include "SFML/Graphics.hpp"
#include "Point.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

class Line : public sf::Drawable
{
public:
    Line(float p1x, float p1y, float p2x, float p2y, sf::Color color = sf::Color::White)
        : line {sf::Vertex(sf::Vector2f(p1x, p1y), color), sf::Vertex(sf::Vector2f(p2x, p2y), color)}{}
    sf::Vertex* getLine() { return line; }
private:
    sf::Vertex line[2];

    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(line, 2, sf::Lines, states);
    }
};

class Renderer
{
protected:
    const unsigned int Width = 400, Height;
    sf::RenderWindow* Window;
    vector<vector<int>> _Map;
    virtual void Draw() = 0;

public: 
    Renderer(vector<vector<int>> &Map) : 
        Height((float)Map.size() / (float)Map[0].size() * Width)
    {
        _Map = Map;
        Window = new sf::RenderWindow(sf::VideoMode(Width, Height), "deneme");
    }
    
    void Run()
    {
        while (Window->isOpen())
        {
            sf::Event event;
            while (Window->pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                    Window->close();
            }

            Window->clear();
            Draw();
            Window->display();

        }    
    }
};

class MapRenderer : public Renderer
{
public:
    MapRenderer(std::vector<std::vector<int>> &map) : Renderer(map) { }
    void drawPoints(vector<Point> &points)
    {
        this->points = points;
    }

private:
    void Draw() override
    {
        float pixelSize = (float)Width / (float)(_Map[0].size());

        for (int i = 0; i < _Map.size(); i++)
        {
            for (int j = 0; j < _Map[0].size(); j++)
            {
                sf::RectangleShape rect(sf::Vector2f(pixelSize, pixelSize));
                rect.setPosition(sf::Vector2f(pixelSize * j, pixelSize * i));

                if (_Map[i][j] == 0)
                    rect.setFillColor(sf::Color::White);
                else
                    rect.setFillColor(sf::Color::Red);

                Window->draw(rect);
            }
        }

        for (const Point &p : points)
        {
            sf::CircleShape circle;
            circle.setRadius(1);
            circle.setPosition(p.x*pixelSize, p.y*pixelSize);
            circle.setFillColor(sf::Color::Blue);
            Window->draw(circle);
        }
    }
private:    
    vector<Point> points;
};

class HeatMapRenderer : public Renderer
{
private:
    const std::vector<std::vector<float>> _passageValues;
    vector<Line> lines;
    const bool drawGrids;
public:
    HeatMapRenderer(const std::vector<std::vector<float>>& passageValues, std::vector<std::vector<int>> &map, bool drawGrids = false) :
        Renderer(map), _passageValues(passageValues), drawGrids(drawGrids) { }

    void drawMatch(Point p1, Point p2)
    {
        float pixelSize = (float)Width / (float)(_Map[0].size());
        lines.push_back({pixelSize*p1.x, pixelSize * p1.y, pixelSize * p2.x, pixelSize*p2.y, sf::Color::Blue});
    }
private:
    void Draw() override
    {
        float pixelSize = (float)Width / (float)(_Map[0].size());

        for (int i = 0; i < _Map.size(); i++)
        {
            for (int j = 0; j < _Map[0].size(); j++)
            {
                sf::RectangleShape rect(sf::Vector2f(pixelSize, pixelSize));
                rect.setPosition(sf::Vector2f(pixelSize * j, pixelSize * i));
                
                if (drawGrids)
                {
                    Line vLine(pixelSize * j, 0, pixelSize * j, Height);
                    Line hLine(0, pixelSize*i, Width, pixelSize*i);
                    Window->draw(vLine);
                    Window->draw(hLine);
                }

                if (_Map[i][j]==1)
                {
                    rect.setFillColor(sf::Color(200,200,200)); 
                    Window->draw(rect);
                }                

                else
                {
                    float normalizedValue = _passageValues[i][j] / min(_Map.size(),_Map[0].size());
                    rect.setFillColor(getColorJet(1-normalizedValue));
                    Window->draw(rect);
                }
            }
        }
        for (auto &line : lines)
            Window->draw(line);
    }

    sf::Color getColorJet(float value)
    {
        float r, g, b;

        if (value < 0.125f)
        {
            r = 0.0f;
            g = 0.0f;
            b = 4.0f * (value + 0.125f);
        }
        else if (value < 0.375f)
        {
            r = 0.0f;
            g = 4.0f * (value - 0.125f);
            b = 1.0f;
        }
        else if (value < 0.625f)
        {
            r = 4.0f * (value - 0.375f);
            g = 1.0f;
            b = 1.0f - 4.0f * (value - 0.375f);
        }
        else if (value < 0.875f)
        {
            r = 1.0f;
            g = 1.0f - 4.0f * (value - 0.625f);
            b = 0.0f;
        }
        else
        {
            r = 1.0f - 4.0f * (value - 0.875f);
            g = 0.0f;
            b = 0.0f;
        }

        unsigned char red = static_cast<unsigned char>(std::round(r * 255.0f));
        unsigned char green = static_cast<unsigned char>(std::round(g * 255.0f));
        unsigned char blue = static_cast<unsigned char>(std::round(b * 255.0f));

        return sf::Color(red, green, blue);
    }

};


class MaskRenderer : public Renderer
{
public:
    MaskRenderer(std::vector<std::vector<bool>>& binaryMask, std::vector<std::vector<int>> &map) :
        Renderer(map), _binaryMask(binaryMask) { }

private:
    void Draw() override
    {
        float pixelSize = (float)Width / (float)(_Map[0].size());

        for (int i = 0; i < _Map.size(); i++)
        {
            for (int j = 0; j < _Map[0].size(); j++)
            {
                sf::RectangleShape rect(sf::Vector2f(pixelSize, pixelSize));
                rect.setPosition(sf::Vector2f(pixelSize * j, pixelSize * i));

                if (_Map[i][j] == 0 && _binaryMask[i][j] == 0)
                    rect.setFillColor(sf::Color::White);
                else if(_Map[i][j] == 0 && _binaryMask[i][j] == 1)
                    rect.setFillColor(sf::Color(250, 100, 0));
                else if(_Map[i][j] == 1 && _binaryMask[i][j] == 0)
                    rect.setFillColor(sf::Color(50, 50, 100));
                else
                    rect.setFillColor(sf::Color::Red);

                Window->draw(rect);
            }
        }
    }
private:
    std::vector<std::vector<bool>> _binaryMask;
};

#endif
