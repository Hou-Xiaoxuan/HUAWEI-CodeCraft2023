#ifndef __TRANS_MAP_H__
#define __TRANS_MAP_H__
#include "model.h"
#include "nav_model.h"
#include <algorithm>
#include <fstream>
#include <utility>
#include <vector>
namespace trans_map
{
using namespace navmesh;
using namespace std;

struct Poly {
    vector<Vertex> points;
    Poly() = default;
    Poly(vector<Vertex> points) : points(std::move(points)) { }
    Poly(const Polygon &poly) : points(poly.vertices) { }
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
bool is_in_polygon(const Poly &poly, Vertex p, bool is_on_edge = true)
{
    bool flag = false;
    for (int i = 0; i < poly.points.size(); ++i)
    {
        int j = (i + 1) % static_cast<int>(poly.points.size());
        Vertex p1 = poly.points[i];
        Vertex p2 = poly.points[j];

        if (is_on_segment(p1, p2, p)) return is_on_edge;

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
        int j = (i + 1) % static_cast<int>(poly.vertices.size());
        sum += poly.vertices[i].x * poly.vertices[j].y - poly.vertices[j].x * poly.vertices[i].y;
    }
    return sum < 0;
}

vector<Poly> polys;

void trans_map(const vector<vector<char>> &ori_map)
{

    vector<vector<char>> maps = vector<vector<char>>(Map::width + 2, vector<char>(Map::height + 2, '#'));
    for (int i = 1; i <= Map::width; ++i)
    {
        for (int j = 1; j <= Map::height; ++j)
        {
            maps[i][j] = ori_map[i][j];
        }
    }

    // 顺时针 四向
    vector<pair<int, int>> dirs = {
        { 0,  1},
        { 1,  0},
        { 0, -1},
        {-1,  0}
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
            return {x * 0.5, (y - 1) * 0.5};
        }
        else if (k == 1)
        {
            return {x * 0.5, y * 0.5};
        }
        else if (k == 0)
        {
            return {(x - 1) * 0.5, y * 0.5};
        }
        else if (k == 3)
        {
            return {(x - 1) * 0.5, (y - 1) * 0.5};
        }
        return {0, 0};
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
        xs.emplace_back();
        ys.emplace_back();
        ks.emplace_back();
        polys.emplace_back();
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
}

vector<Polygon> tree;

void get_polygon()
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

bool check_segment_valid(const Segment &line, const Polygon &poly)
{
    for (int i = 1; i < 3; ++i)
    {
        Vertex p = {line.a.x + (line.b.x - line.a.x) * i / 3, line.a.y + (line.b.y - line.a.y) * i / 3};
        if (not is_in_polygon(poly, p, false)) return false;

        for (auto &hole : poly.holes)
        {
            if (is_in_polygon(hole, p))
            {
                return false;
            }
        }
    }
    return true;
}


vector<Segment> danger_line;
vector<Segment> stop_line;

void get_danger_line()
{
    double danger_dis = 1.1;
    double stop_dis = ConVar::robot_radius * 2 + 1e-6;
    for (auto &poly : result)
    {
        vector<Vertex> all_point = poly.vertices;
        vector<Segment> all_segment;
        vector<Segment> tmp_danger_line;
        vector<Segment> tmp_stop_line;
        for (auto &hole : poly.holes)
        {
            all_point.insert(all_point.end(), hole.vertices.begin(), hole.vertices.end());
        }
        for (int i = 0; i < poly.vertices.size(); ++i)
        {
            all_segment.emplace_back(poly.vertices[i], poly.vertices[(i + 1) % poly.vertices.size()]);
        }
        for (auto &hole : poly.holes)
        {
            for (int i = 0; i < hole.vertices.size(); ++i)
            {
                all_segment.emplace_back(hole.vertices[i], hole.vertices[(i + 1) % hole.vertices.size()]);
            }
        }


        for (int i = 0; i < all_point.size(); ++i)
        {
            for (int j = i + 1; j < all_point.size(); ++j)
            {
                double dis = Vec2(all_point[i], all_point[j]).length();
                if (dis <= stop_dis)
                {
                    tmp_stop_line.push_back(all_point[i] < all_point[j]
                            ? Segment(all_point[i], all_point[j])
                            : Segment(all_point[j], all_point[i]));
                }
                else if (dis <= danger_dis)
                {
                    tmp_danger_line.push_back(all_point[i] < all_point[j]
                            ? Segment(all_point[i], all_point[j])
                            : Segment(all_point[j], all_point[i]));
                }
            }
        }

        for (int i = 0; i < all_point.size(); ++i)
        {
            for (int j = 0; j < all_segment.size(); ++j)
            {
                Vertex p = point_point_to_segment(all_point[i], all_segment[j]);
                if (point_on_line(all_point[i], all_segment[j])) continue;
                double dis = Vec2(all_point[i], p).length();
                if (dis <= stop_dis)
                {
                    tmp_stop_line.push_back(
                        all_point[i] < p ? Segment(all_point[i], p) : Segment(p, all_point[i]));
                }
                else if (dis <= danger_dis)
                {
                    tmp_danger_line.push_back(
                        all_point[i] < p ? Segment(all_point[i], p) : Segment(p, all_point[i]));
                }
            }
        }

        sort(tmp_stop_line.begin(), tmp_stop_line.end());
        tmp_stop_line.erase(unique(tmp_stop_line.begin(), tmp_stop_line.end()), tmp_stop_line.end());
        for (auto &line : tmp_stop_line)
        {
            if (check_segment_valid(line, poly))
            {
                stop_line.push_back(line);
            }
        }

        sort(tmp_danger_line.begin(), tmp_danger_line.end());
        tmp_danger_line.erase(
            unique(tmp_danger_line.begin(), tmp_danger_line.end()), tmp_danger_line.end());
        for (auto &line : tmp_danger_line)
        {
            if (check_segment_valid(line, poly))
            {
                danger_line.push_back(line);
            }
        }
    }

    for (int i = 0; i < danger_line.size(); ++i)
    {
        for (int j = i + 1; j < danger_line.size(); ++j)
        {
            if (Segment::is_cross_2(danger_line[i], danger_line[j]))
            {
                if (danger_line[i].length() < danger_line[j].length())
                {
                    swap(danger_line[j], danger_line.back());
                    danger_line.pop_back();
                    --j;
                }
                else
                {
                    swap(danger_line[i], danger_line.back());
                    danger_line.pop_back();
                    --i;
                    break;
                }
            }
        }
    }
    for (int i = 0; i < stop_line.size(); ++i)
    {
        for (int j = 0; j < danger_line.size(); ++j)
        {
            if (Segment::is_cross_2(stop_line[i], danger_line[j]))
            {
                swap(danger_line[j], danger_line.back());
                danger_line.pop_back();
                --j;
            }
        }
    }
}

vector<vector<double>> nearest_obstacle(Map::width + 2, vector<double>(Map::height + 2, 1e9));

void get_nearest_obstacle()
{
    for (int i = 1; i <= Map::width; ++i)
    {
        for (int j = 1; j <= Map::height; ++j)
        {
            if (meta.map[i][j] == '#') continue;
            Vertex center = {(i - 1) * 0.5 + 0.25, (j - 1) * 0.5 + 0.25};
            for (auto &poly : polys)
            {
                const auto &points = poly.points;
                for (int k = 0; k < points.size(); ++k)
                {
                    nearest_obstacle[i][j] = min(nearest_obstacle[i][j],
                        dis_point_to_segment(center, Segment {points[k], points[(k + 1) % points.size()]}));
                }
            }
        }
    }
}

void test_print()
{
    fstream fout("./Robot/map.txt");
    if (!fout.is_open())
    {
        cout << "open file failed" << endl;
        return;
    }
    fout << result.size() << endl;
    for (const auto &p : result)
    {
        fout << p.vertices.size() << endl;
        for (auto &v : p.vertices)
        {
            fout << v.x << endl;
            fout << v.y << endl;
        }
        fout << p.holes.size() << endl;
        for (const auto &hole : p.holes)
        {
            fout << hole.vertices.size() << endl;
            for (auto &v : hole.vertices)
            {
                fout << v.x << endl;
                fout << v.y << endl;
            }
        }
    }

    fout << danger_line.size() << endl;
    for (auto &line : danger_line)
    {
        fout << line.a.x << endl;
        fout << line.a.y << endl;
        fout << line.b.x << endl;
        fout << line.b.y << endl;
    }

    fout << stop_line.size() << endl;
    for (auto &line : stop_line)
    {
        fout << line.a.x << endl;
        fout << line.a.y << endl;
        fout << line.b.x << endl;
        fout << line.b.y << endl;
    }

    fout.close();
}
vector<Polygon> init()
{
    /*识别 .#  #.
          #.  .#
    两种情况，填上一个.
    */
    /* 识别

    */
    for (int i = 2; i <= 100; i++)
        for (int j = 1; j <= 100; j++)
        {
            if (meta.map[i][j] == '#' and meta.map[i - 1][j] == '.' and meta.map[i][j - 1] == '.'
                and meta.map[i - 1][j - 1] == '#')
                model::meta.map[i - 1][j] = '#';
            if (meta.map[i][j] == '.' and meta.map[i - 1][j] == '#' and meta.map[i][j - 1] == '#'
                and meta.map[i - 1][j - 1] == '.')
                model::meta.map[i - 1][j - 1] = '#';
        }

    trans_map(meta.map);
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
    get_polygon();
    get_result(tree.back(), 0);
    get_danger_line();
    get_nearest_obstacle();
#ifdef DEBUG
    test_print();
#endif

    // // 根据danger_line填充修改地图
    // for (auto &line : danger_line)
    // {
    //     // 填充a->b上，很纵坐标为0.25倍数的点
    //     double x = line.a.x;
    //     double y = line.a.y;
    //     double k = (line.b.y - line.a.y) / (line.b.x - line.a.x);
    //     double b = line.a.y - k * line.a.x;
    //     do{
    //         int x1 = (int)(x * 2);
    //         int y1 = (int)(y * 2);
    //         if(meta.map[x1][y1] == '.')
    //             model::meta.map[x1][y1] = '&';
    //         x += 0.25;
    //         y = k * x + b;
    //     }while(x < line.b.x);
    // }
    // 打印修改后的地图
    for (auto &i : model::meta.map)
    {
        for (char j : i)
        {
            cerr << j;
        }
        cerr << endl;
    }

    return result;
}
}    // namespace
#endif