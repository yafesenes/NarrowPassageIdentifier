#ifndef CONVEX_CLASS_H
#define CONVEX_CLASS_H

#include "Point.h"
#include <vector>
#include <algorithm>
#include "TicToc.h"

using namespace std;

class ConvexFinder
{
private:
    // Noktaları sıralamak için kullanılacak karşılaştırma fonksiyonu
    static bool compare(Point p1, Point p2) {
        return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
    }

    // Çapraz çarpımı hesaplamak için bir yardımcı fonksiyon
    static int cross(const Point &O, const Point &A, const Point &B) {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

public:
    static pair<Point, Point> ConvexToRect(const vector<Point> &Convex)
    {
        Point TopLeft = Convex[0];
        Point BotRight = Convex[0];

        for (const Point &p:Convex)
        {
            if (p.x<TopLeft.x)
                TopLeft.x = p.x;

            if (p.y<TopLeft.y)
                TopLeft.y = p.y;

            if (p.x>BotRight.x)
                BotRight.x = p.x;

            if (p.y>BotRight.y)
                BotRight.y = p.y;
        }

        return make_pair(TopLeft, BotRight);
    }

    static vector<Point> ConvexHull(vector<Point> P) {
        tic("ConvexHull");
        int n = P.size(), k = 0;
        vector<Point> H(2 * n);

        // Sırala
        sort(P.begin(), P.end(), compare);

        // Alt dışbükey zarfı oluştur
        for (int i = 0; i < n; ++i) {
            while (k >= 2 && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
            H[k++] = P[i];
        }

        // Üst dışbükey zarfı oluştur
        for (int i = n - 2, t = k + 1; i >= 0; i--) {
            while (k >= t && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
            H[k++] = P[i];
        }

        H.resize(k - 1);

        toc("ConvexHull");
        return H;
    }

    static void ClusterConvexPolygon(const vector<Point> &polygon, const vector<Point> &points, pair<vector<Point>, vector<Point>>& PolygonPoints)
    {
        size_t n = polygon.size();

        for(const Point &point : points) {
            int sign = 0;
            bool inside = true;

            for(size_t i = 0; i < n; ++i) {
                size_t j = (i + 1) % n;
                int cp = cross(polygon[i], polygon[j], point);

                if(cp == 0) continue; // Nokta kenarda olabilir

                int newSign = (cp > 0) ? 1 : -1;

                if(sign == 0) {
                    sign = newSign;
                } else if(sign != newSign) {
                    inside = false;
                    break;
                }
            }

            if(inside) {
                PolygonPoints.second.push_back(point);
            } else {
                PolygonPoints.first.push_back(point);
            }
        }
    }

    // Konveks bir zarfın içerisinde bir noktanın olup olmadığını kontrol eder.
    static bool isInsideConvex(const std::vector<Point> &convexHull, const Point &point) {
        size_t n = convexHull.size();
        for (size_t i = 0; i < n; ++i) {
            // Her üç ardışık nokta için çapraz çarpım hesapla
            size_t j = (i + 1) % n;

            // Eğer çapraz çarpım negatifse, nokta dışarıdadır.
            if (cross(convexHull[i], convexHull[j], point) < 0) {
                return false;
            }
        }
        return true; // Tüm çapraz çarpımlar pozitifse, nokta içeridedir.
    }
};

#endif