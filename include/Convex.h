#ifndef CONVEX_CLASS_H
#define CONVEX_CLASS_H

#include "Point.h"
#include <vector>
#include <algorithm>

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
    static vector<Point> ConvexHull(vector<Point> P) {
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
        return H;
    }

};

#endif