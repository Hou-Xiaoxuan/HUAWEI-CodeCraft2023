#ifndef __TRANS_MAP_H__
#define __TRANS_MAP_H__
#include "model.h"
#include "nav_model.h"
#include <vector>
namespace trans_map
{
using namespace navmesh;
using namespace std;

struct Poly {
    vector<Vertex> points;
    Poly() = default;
    Poly(Polygon poly) { points = poly.vertices; }
};

// 三态函数，判断两个double在eps精度下的大小关系
int dcmp(double x)
{
    if (fabs(x) < 1e-6)
        return 0;
    else
        return x < 0 ? -1 : 1;
}

// 判断点Q是否在P1和P2的线段上
bool is_on_segment(const Vertex &p1, const Vertex &p2, const Vertex &q)
{
    // 前一个判断点Q在P1P2直线上 后一个判断在P1P2范围上
    return dcmp(Vec2(q, p1) ^ Vec2(q, p2)) == 0 and dcmp(Vec2(q, p1) * Vec2(q, p2)) <= 0;
}

// 判断点P在多边形内-射线法
bool is_in_polygon(const Poly &poly, Vertex p)
{
    bool flag = false;
    for (int i = 0; i < poly.points.size(); ++i)
    {
        int j = (i + 1) % poly.points.size();
        Vertex p1 = poly.points[i];
        Vertex p2 = poly.points[j];

        if (is_on_segment(p1, p2, p)) return true;

        if ((dcmp(p1.y - p.y) > 0 != dcmp(p2.y - p.y) > 0)
            and dcmp(p.x - (p.y - p1.y) * (p1.x - p2.x) / (p1.y - p2.y) - p1.x) < 0)
        {
            flag = !flag;
        }
    }
    return flag;
}

bool is_clockwise(const Polygon &poly)
{
    double sum = 0;
    for (int i = 0; i < poly.vertices.size(); ++i)
    {
        int j = (i + 1) % poly.vertices.size();
        sum += poly.vertices[i].x * poly.vertices[j].y - poly.vertices[j].x * poly.vertices[i].y;
    }
    return sum < 0;
}


vector<Poly> trans_map(const vector<vector<char>> &ori_map)
{
    vector<Poly> polys;

    vector<vector<char>> maps = vector<vector<char>>(Map::width + 2, vector<char>(Map::height + 2, '#'));
    for (int i = 1; i <= Map::width; ++i)
    {
        for (int j = 1; j <= Map::height; ++j)
        {
            maps[i][j] = ori_map[i][j];
        }
    }

    auto print_map = [&]() {
        for (int i = 1; i <= Map::width; ++i)
        {
            for (int j = 1; j <= Map::height; ++j)
            {
                cerr << maps[i][j];
            }
            cerr << endl;
        }
    };
    print_map();

    // 顺时针 四向
    vector<pair<int, int>> dirs = {
        { 0,  1},
        { 1,  0},
        { 0, -1},
        {-1,  0},
    };

    vector<vector<vector<int>>> map_status = vector<vector<vector<int>>>(
        Map::width + 2, vector<vector<int>>(Map::height + 2, vector<int>(4, 0)));
    for (int i = 0; i <= Map::width + 1; ++i)
    {
        for (int j = 0; j <= Map::height + 1; ++j)
        {
            if (maps[i][j] != '#') continue;


            for (int k = 0; k < 4; ++k)
            {
                int nx = i + dirs[k].first;
                int ny = j + dirs[k].second;
                if (nx < 0 or nx > Map::width + 1 or ny < 0 or ny > Map::height + 1)
                {
                    continue;
                }

                if (maps[nx][ny] == '#') continue;

                map_status[i][j][k] = 1;
            }
        }
    }
    auto limit_4 = [](int k) -> int {
        return (k + 4) % 4;
    };
    auto trans_point = [](int x, int y, int k) -> Vertex {
        if (k == 2)
        {
            return Vertex(x * 0.5, (y - 1) * 0.5);
        }
        else if (k == 1)
        {
            return Vertex(x * 0.5, y * 0.5);
        }
        else if (k == 0)
        {
            return Vertex((x - 1) * 0.5, y * 0.5);
        }
        else if (k == 3)
        {
            return Vertex((x - 1) * 0.5, (y - 1) * 0.5);
        }
        return Vertex(0, 0);
    };
    // dingdian
    vector<vector<int>> xs;
    vector<vector<int>> ys;
    vector<vector<int>> ks;

    while (true)
    {

        // find first ret #
        int x = 0, y = 0, dir = 0;
        for (int i = 0; i <= Map::width + 1 and (!x and !y); ++i)
        {
            for (int j = 0; j <= Map::height + 1 and (!x and !y); ++j)
            {
                if (maps[i][j] != '#')
                {
                    continue;
                }

                for (int k = 0; k < 4; ++k)
                {
                    if (map_status[i][j][k] == 1)
                    {
                        x = i;
                        y = j;
                        dir = k;
                        break;
                    }
                }
            }
        }
        if (!x and !y) break;
        int startx = x, starty = y, startdir = dir;
        xs.push_back(vector<int>());
        ys.push_back(vector<int>());
        ks.push_back(vector<int>());
        polys.push_back(Poly());
        while (true)
        {
            bool stop_flag = true;
            map_status[x][y][dir] = 2;
            vector<int> nx = {x + dirs[limit_4(dir - 1)].first, x + dirs[limit_4(dir + 1)].first,
                x + dirs[limit_4(dir - 1)].first + dirs[dir].first,
                x + dirs[limit_4(dir + 1)].first + dirs[dir].first, x, x};
            vector<int> ny = {y + dirs[limit_4(dir - 1)].second, y + dirs[limit_4(dir + 1)].second,
                y + dirs[limit_4(dir - 1)].second + dirs[dir].second,
                y + dirs[limit_4(dir + 1)].second + dirs[dir].second, y, y};
            vector<int> ndir
                = {dir, dir, limit_4(dir + 1), limit_4(dir - 1), limit_4(dir - 1), limit_4(dir + 1)};

            for (int i = 0; i < 6; ++i)
            {
                if (nx[i] < 0 or nx[i] > Map::width + 1 or ny[i] < 0 or ny[i] > Map::height + 1)
                {
                    continue;
                }
                if (maps[nx[i]][ny[i]] != '#')
                {
                    continue;
                }
                if (map_status[nx[i]][ny[i]][ndir[i]] != 1)
                {
                    continue;
                }

                if (dir != ndir[i])
                {
                    xs.back().push_back(x);
                    ys.back().push_back(y);
                    ks.back().push_back(limit_4(dir + i % 2));
                    polys.back().points.push_back(trans_point(x, y, limit_4(dir + i % 2)));
                    dir = ndir[i];
                }
                x = nx[i];
                y = ny[i];
                stop_flag = false;
                break;
            }

            if (stop_flag)
            {
                for (int i = 2; i < 6; ++i)
                {
                    if (nx[i] < 0 or nx[i] > Map::width + 1 or ny[i] < 0 or ny[i] > Map::height + 1)
                    {
                        continue;
                    }
                    if (nx[i] == startx and ny[i] == starty and ndir[i] == startdir)
                    {
                        xs.back().push_back(x);
                        ys.back().push_back(y);
                        ks.back().push_back(limit_4(dir + i % 2));
                        polys.back().points.push_back(trans_point(x, y, limit_4(dir + i % 2)));
                        break;
                    }
                }
                break;
            }
        }
    }
    return polys;
}

vector<Polygon> tree;

void get_polygon(const vector<Poly> &polys)
{
    tree = vector<Polygon>(polys.size());
    for (int i = 0; i < polys.size(); ++i)
    {
        tree[i].vertices = polys[i].points;
        for (int j = i + 1; j < polys.size(); ++j)
        {
            if (is_in_polygon(polys[j], polys[i].points[0]))
            {
                tree[j].holes.push_back(tree[i]);
                break;
            }
        }
    }
}

vector<Polygon> result;
void get_result(const Polygon &p, int step)
{
    if (step % 2 == 0)
    {
        result.push_back(p);
        if (not is_clockwise(result.back()))
        {
            reverse(result.back().vertices.begin(), result.back().vertices.end());
        }
        for (auto &h : result.back().holes)
        {
            if (is_clockwise(h))
            {
                reverse(h.vertices.begin(), h.vertices.end());
            }
        }
    }

    for (const auto &h : p.holes)
    {
        get_result(h, step + 1);
    }
}


void test_print()
{
    for (const auto &p : result)
    {
        cerr << p.holes.size() << endl;
        cerr << p.vertices.size() << endl;
        for (auto &v : p.vertices)
        {
            cerr << v.x << endl;
            cerr << v.y << endl;
        }
        for (auto hole : p.holes)
        {
            cerr << 0 << endl;
            cerr << hole.vertices.size() << endl;
            for (auto &v : hole.vertices)
            {
                cerr << v.x << endl;
                cerr << v.y << endl;
            }
        }
    }
}
vector<Vertex> danger_line;


vector<Polygon> solve()
{
    auto polys = trans_map(meta.map);
    sort(polys.begin(), polys.end(), [](const Poly &b, const Poly &a) {
        Vertex a_p = a.points[0];
        Vertex b_p = b.points[0];
        if (is_in_polygon(b, a_p))
        {
            return false;
        }
        else if (is_in_polygon(a, b_p))
        {
            return true;
        }
        return false;
    });
    get_polygon(polys);
    get_result(tree.back(), 0);
    test_print();

    return result;
}
}    // namespace
#endif