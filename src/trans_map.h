#ifndef __TRANS_MAP_H__
#define __TRANS_MAP_H__
#include "args.h"
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


struct Pos {
    int index_x = -1;
    int index_y = -1;
    Vertex pos = {-1, -1};
};

// 找到当前点所在的meta.map中的正方形 返回值 正方形x轴index y轴index 这个点坐标
Pos current_pos(const Vertex &v)
{

    for (int i = 2 * int(v.x); i <= 2 * int(v.x) + 2; ++i)
    {
        for (int j = 2 * int(v.y); j <= 2 * int(v.y) + 2; ++j)
        {
            if (v.x <= i * 0.5 and v.x >= (i - 1) * 0.5 and v.y <= j * 0.5 and v.y >= (j - 1) * 0.5)
                return {i, j, v};
        }
    }
    if (_USE_LOG_)
    {
        cerr << "[error][current_pos] can't find current pos with " << v << endl;
        throw "can't find current pos";
    }
}

struct Poly {
    vector<Vertex> points;
    Poly() = default;
    Poly(vector<Vertex> points) : points(std::move(points)) { }
    Poly(const Polygon &poly) : points(poly.vertices) { }
};

// 获得正方形重心
inline Vertex get_center(int i, int j) { return {(i - 1) * 0.5 + 0.25, (j - 1) * 0.5 + 0.25}; }

// 顺时针 四向
vector<pair<int, int>> dirs = {
    { 0,  1},
    { 1,  0},
    { 0, -1},
    {-1,  0},
    {-1, -1},
    { 1, -1},
    { 1,  1},
    {-1,  1}
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

struct SkipSegment : Segment {
    Vertex choice_a;
    Vertex choice_b;
    SkipSegment(const Segment &s) : Segment(s) { }
    SkipSegment(const Segment &s, const Vertex &a, const Vertex &b) : Segment(s), choice_a(a), choice_b(b)
    { }
    Vertex get_skip_vertex(const Vertex &v, const pair<int, int> &dir)
    {
        if (Vertex::distance(v, choice_a) < Vertex::distance(v, choice_b))
            return choice_b;
        else if (Vertex::distance(v, choice_a) > Vertex::distance(v, choice_b))
            return choice_a;
        else
        {
            // 用ab做向量 dir做向量 比較向量夾角
            Vec2 ab = Vec2(choice_a, choice_b);
            Vec2 d;
            d.x = dir.first;
            d.y = dir.second;
            if (ab * d > 0)
            {
                return choice_a;
            }
            else
            {
                return choice_b;
            }
        }
    }
};
vector<SkipSegment> skip_line;
vector<Segment> danger_line;
vector<Segment> stop_line;

void get_skip_line()
{
    // 识别map里的形状及其变形
    /* ...#
       ....
       #...
    */
    for (int i = 1; i + 3 <= Map::width; i++)
        for (int j = 1; j + 2 <= Map::height; j++)
        {
            if (meta.map[i][j] == '#' and meta.map[i + 3][j + 2] == '#')
            {
                bool flag = true;
                for (int _i = i; _i <= i + 3 and flag; _i++)
                    for (int _j = j; _j <= j + 2 and flag; _j++)
                    {
                        if (_i == i and _j == j) continue;
                        if (_i == i + 3 and _j == j + 2) continue;
                        if (meta.map[_i][_j] == '#') flag = false;
                    }
                if (flag)
                {
                    auto a = get_center(i, j);
                    auto b = get_center(i + 3, j + 2);
                    skip_line.emplace_back(Segment {
                        Vertex {a.x + 0.25, a.y + 0.25},
                         Vertex {b.x - 0.25, b.y - 0.25}
                    });
                    skip_line.back().choice_a = {a.x, b.y};
                    skip_line.back().choice_b = {b.x, a.y};
                }
            }
        }
    /*  #...
        ....
        ...#
    */
    for (int i = 1; i + 3 <= Map::width; i++)
        for (int j = 1; j + 2 <= Map::height; j++)
        {
            if (meta.map[i][j + 2] == '#' and meta.map[i + 3][j] == '#')
            {
                bool flag = true;
                for (int _i = i; _i <= i + 3 and flag; _i++)
                    for (int _j = j; _j <= j + 2 and flag; _j++)
                    {
                        if (_i == i and _j == j + 2) continue;
                        if (_i == i + 3 and _j == j) continue;
                        if (meta.map[_i][_j] == '#') flag = false;
                    }
                if (flag)
                {
                    auto a = get_center(i, j + 2);
                    auto b = get_center(i + 3, j);
                    skip_line.emplace_back(Segment {
                        Vertex {a.x + 0.25, a.y - 0.25},
                         Vertex {b.x - 0.25, b.y + 0.25}
                    });
                    skip_line.back().choice_a = {a.x, b.y};
                    skip_line.back().choice_b = {b.x, a.y};
                }
            }
        }


    /*  #..
        ...
        ...
        ..#
    */
    for (int i = 1; i + 2 <= Map::width; i++)
        for (int j = 1; j + 3 <= Map::height; j++)
        {
            if (meta.map[i][j + 3] == '#' and meta.map[i + 2][j] == '#')
            {
                bool flag = true;
                for (int _i = i; _i <= i + 2 and flag; _i++)
                    for (int _j = j; _j <= j + 3 and flag; _j++)
                    {
                        if (_i == i and _j == j + 3) continue;
                        if (_i == i + 2 and _j == j) continue;
                        if (meta.map[_i][_j] == '#') flag = false;
                    }
                if (flag)
                {
                    auto a = get_center(i, j + 3);
                    auto b = get_center(i + 2, j);
                    skip_line.emplace_back(Segment {
                        Vertex {a.x + 0.25, a.y - 0.25},
                         Vertex {b.x - 0.25, b.y + 0.25}
                    });
                    skip_line.back().choice_a = {a.x, b.y};
                    skip_line.back().choice_b = {b.x, a.y};
                }
            }
        }
    /*  ..#
        ...
        ...
        #..
    */
    for (int i = 1; i + 2 <= Map::width; i++)
        for (int j = 1; j + 3 <= Map::height; j++)
        {
            if (meta.map[i][j] == '#' and meta.map[i + 2][j + 3] == '#')
            {
                bool flag = true;
                for (int _i = i; _i <= i + 2 and flag; _i++)
                    for (int _j = j; _j <= j + 3 and flag; _j++)
                    {
                        if (_i == i and _j == j) continue;
                        if (_i == i + 2 and _j == j + 3) continue;
                        if (meta.map[_i][_j] == '#') flag = false;
                    }
                if (flag)
                {
                    auto a = get_center(i, j);
                    auto b = get_center(i + 2, j + 3);
                    skip_line.emplace_back(Segment {
                        Vertex {a.x + 0.25, a.y + 0.25},
                         Vertex {b.x - 0.25, b.y - 0.25}
                    });
                    skip_line.back().choice_a = {a.x, b.y};
                    skip_line.back().choice_b = {b.x, a.y};
                }
            }
        }
}

void get_danger_line()
{
    double danger_dis = 1.0 + 1e-6;
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

bool check_expand_danger(const Vertex &a, const Vertex &b)
{
    Pos pos_a = current_pos(a);
    if (meta.map[pos_a.index_x][pos_a.index_y] == '#') return false;
    Pos pos_b = current_pos(b);
    if (meta.map[pos_b.index_x][pos_b.index_y] == '#') return false;
    return true;
}

void expand_danger_line()
{
    // TODO 檢查
    vector<Segment> tmp_danger_line;
    for (auto &line : danger_line)
    {
        Vertex center = {(line.a.x + line.b.x) * 0.5, (line.a.y + line.b.y) * 0.5};
        if (abs(line.a.x - line.b.x) < 1e-6)
        {
            Vertex tmp_1 = {center.x - 0.25, center.y + 0.25};
            Vertex tmp_2 = {center.x - 0.25, center.y - 0.25};
            if (check_expand_danger(tmp_1, tmp_2))
            {
                tmp_danger_line.push_back({
                    Vertex {line.a.x - 0.5, line.a.y},
                     Vertex {line.b.x - 0.5, line.b.y}
                });
            }
            tmp_1 = {center.x + 0.25, center.y + 0.25};
            tmp_2 = {center.x + 0.25, center.y - 0.25};
            if (check_expand_danger(tmp_1, tmp_2))
            {
                tmp_danger_line.push_back({
                    Vertex {line.a.x + 0.5, line.a.y},
                     Vertex {line.b.x + 0.5, line.b.y}
                });
            }
        }
        else
        {
            Vertex tmp_1 = {center.x + 0.25, center.y - 0.25};
            Vertex tmp_2 = {center.x - 0.25, center.y - 0.25};
            if (check_expand_danger(tmp_1, tmp_2))
            {
                tmp_danger_line.push_back({
                    Vertex {line.a.x, line.a.y - 0.5},
                     Vertex {line.b.x, line.b.y - 0.5}
                });
            }
            tmp_1 = {center.x + 0.25, center.y + 0.25};
            tmp_2 = {center.x - 0.25, center.y + 0.25};
            if (check_expand_danger(tmp_1, tmp_2))
            {
                tmp_danger_line.push_back({
                    Vertex {line.a.x, line.a.y + 0.5},
                     Vertex {line.b.x, line.b.y + 0.5}
                });
            }
        }
    }
    // 合併danger_line 和 tmp_danger_line 排序 去重
    danger_line.insert(danger_line.end(), tmp_danger_line.begin(), tmp_danger_line.end());
    sort(danger_line.begin(), danger_line.end());
    danger_line.erase(unique(danger_line.begin(), danger_line.end()), danger_line.end());
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
    fstream fout("./map.txt");
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


    fout << skip_line.size() << endl;
    for (auto &line : skip_line)
    {
        fout << line.a.x << endl;
        fout << line.a.y << endl;
        fout << line.b.x << endl;
        fout << line.b.y << endl;
    }

    fout.close();
}


vector<vector<vector<double>>> valid_map(Map::width + 2,
    vector<vector<double>>(Map::height + 2, vector<double>(8, 0.0)));


void get_valid_map()
{
    for (int i = 1; i <= Map::width; ++i)
    {
        for (int j = 1; j <= Map::height; ++j)
        {
            if (meta.map[i][j] == '#') continue;
            Vertex now_center = get_center(i, j);
            for (int k = 0; k < 8; ++k)
            {
                int nx = i + dirs[k].first;
                int ny = j + dirs[k].second;
                if (meta.map[nx][ny] == '#') continue;
                Vertex ncenter = get_center(nx, ny);
                Segment seg = {now_center, ncenter};
                double tmp = 1e9;
                for (const auto &poly : trans_map::polys)
                {
                    const auto &points = poly.points;
                    for (int _k = 0; _k < points.size(); ++_k)
                    {
                        tmp = min(
                            tmp, Segment::distance(seg, {points[_k], points[(_k + 1) % points.size()]}));
                    }
                }
                valid_map[i][j][k] = tmp;
            }
        }
    }
}

vector<Polygon> init()
{

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
    expand_danger_line();
    get_nearest_obstacle();
    get_valid_map();
    get_skip_line();
    if (_USE_LOG_)
    {
        test_print();
        for (auto &i : model::meta.map)
        {
            for (char j : i)
            {
                cerr << j;
            }
            cerr << endl;
        }
    }
    return result;
}
}    // namespace
#endif