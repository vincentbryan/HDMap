//
// Created by iceytan on 18-11-26.
//

#ifndef POLYGONSCAN_POLYGONSCAN_H
#define POLYGONSCAN_POLYGONSCAN_H

#include <cstdio>
#include <vector>
#include <limits>
#include <chrono>


namespace PolygonScan {
    using TimePoint = std::chrono::steady_clock::time_point;

    struct Edge {
        int ymax;
        float x;
        float dx;
        Edge *next;
    };

    struct Point {
        int x;
        int y;

        Point(int x, int y) {
            this->x = x;
            this->y = y;
        }
    };


    static std::vector<Point> polygonScan(std::vector<Point> &vertices) {
        TimePoint _tp;

        Edge *AET;
        std::vector<Point> res;
        if (vertices.size() < 3)
            return res;

        int maxX = std::numeric_limits<int>::min();
        int maxY = std::numeric_limits<int>::min();

        for (auto &vertice : vertices) {
            if (vertice.y > maxY) maxY = vertice.y;
            if (vertice.x > maxX) maxX = vertice.x;
        }

        Edge *pET[maxY + 1];
        for (int i = 0; i < maxY; i++) {
            pET[i] = new Edge();
            pET[i]->next = nullptr;
        }
        AET = new Edge();
        AET->next = nullptr;

        for (int i = 0; i < vertices.size(); i++) {
            int x0 = vertices[(i - 1 + vertices.size()) % vertices.size()].x;
            int x1 = vertices[i].x;
            int x2 = vertices[(i + 1) % vertices.size()].x;
            int x3 = vertices[(i + 2) % vertices.size()].x;
            int y0 = vertices[(i - 1 + vertices.size()) % vertices.size()].y;
            int y1 = vertices[i].y;
            int y2 = vertices[(i + 1) % vertices.size()].y;
            int y3 = vertices[(i + 2) % vertices.size()].y;
            if (y1 == y2)
                continue;

            int ymin = y1 > y2 ? y2 : y1;
            int ymax = y1 > y2 ? y1 : y2;
            float x = y1 > y2 ? x2 : x1;
            float dx = (x1 - x2) * 1.0f / (y1 - y2);

            if (((y1 < y2) && (y1 > y0)) || ((y2 < y1) && (y2 > y3))) {
                ymin++;
                x += dx;
            }

            Edge *p = new Edge();
            p->ymax = ymax;
            p->x = x;
            p->dx = dx;
            p->next = pET[ymin]->next;
            pET[ymin]->next = p;
        }

        for (int i = 0; i < maxY; i++) {
            while (pET[i]->next) {
                Edge *pInsert = pET[i]->next;
                Edge *p = AET;
                while (p->next) {
                    if (pInsert->x > p->next->x) {
                        p = p->next;
                        continue;
                    }
                    if (pInsert->x == p->next->x && pInsert->dx > p->next->dx) {
                        p = p->next;
                        continue;
                    }
                    break;
                }
                pET[i]->next = pInsert->next;
                pInsert->next = p->next;
                p->next = pInsert;
            }

            Edge *p = AET;
            while (p->next && p->next->next) {
                for (int x = p->next->x; x < p->next->next->x; x++) {
                    res.emplace_back(x, i);
                }
                p = p->next->next;
            }
            p = AET;
            while (p->next) {
                if (p->next->ymax == i) {
                    Edge *pDelete = p->next;
                    p->next = pDelete->next;
                    pDelete->next = nullptr;
                    delete pDelete;
                } else {
                    p = p->next;
                }
            }
            p = AET;
            while (p->next) {
                p->next->x += p->next->dx;
                p = p->next;
            }
        }
        TimePoint _t = std::chrono::steady_clock::now();
        printf("%f", std::chrono::duration_cast<std::chrono::duration<double>>(_t - _tp).count());

        return (res);
    }
}


#endif //POLYGONSCAN_POLYGONSCAN_H
