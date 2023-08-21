#ifndef RENDERER_CLASS_H
#define RENDERER_CLASS_H
#include "SFML/Graphics.hpp"
#include "Point.h"
#include <vector>
#include <iostream>

using namespace std;

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
