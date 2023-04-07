#ifndef __FIND_PATH_SQURE_H__
#define __FIND_PATH_SQURE_H__
#include "const.h"
#include "nav_model.h"
#include "trans_map.h"
#include <algorithm>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace find_path_square
{
using namespace std;
using namespace navmesh;


struct Pos {
    int index_x = -1;
    int index_y = -1;
    Vertex pos = {-1, -1};
};

vector<pair<int, int>> dirs = {
    { 0,  1},
    { 1,  0},
    { 0, -1},
    {-1,  0},
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
    throw "current_pos error";
}


// 获得正方形重心
inline Vertex get_center(int i, int j) { return {(i - 1) * 0.5 + 0.25, (j - 1) * 0.5 + 0.25}; }


// 优先队列排序函数
auto cmp_pos = [](const pair<Pos, int> &a, const pair<Pos, int> &b) -> bool {
    return a.second > b.second
        or (a.second == b.second
            and trans_map::nearest_obstacle[a.first.index_x][a.first.index_y]
                < trans_map::nearest_obstacle[b.first.index_x][b.first.index_y]);
};


vector<vector<Vertex>> proper_pos;
vector<vector<Pos>> pre;

Vertex start;
Vertex target;
bool have_good;

void get_proper_pos()
{
    double limit_dis = ConVar::robot_radius_goods + 0.05;

    for (int i = 1; i <= Map::width; ++i)
    {
        for (int j = 1; j <= Map::height; ++j)
        {
            if (meta.map[i][j] == '#') continue;

            Vertex center = proper_pos[i][j];
            Vertex nearest_vertex;
            double nearest_dis = 1e9;
            for (const auto &poly : trans_map::polys)
            {
                const auto &points = poly.points;
                for (int k = 0; k < points.size(); ++k)
                {
                    Segment line = {points[k], points[(k + 1) % points.size()]};
                    double tmp_dis = dis_point_to_segment(center, line);
                    if (tmp_dis < nearest_dis)
                    {
                        nearest_dis = tmp_dis;
                        nearest_vertex = point_point_to_segment(center, line);
                    }
                }
            }

            if (nearest_dis < limit_dis)
            {
                Vec2 v = {nearest_vertex, center};
                // c 沿着 v 方向移动 limit_path
                // XXX v 理论上应该不可能为 0
                Vec2 v_unit = {v.x / v.length(), v.y / v.length()};
                Vertex new_c
                    = {nearest_vertex.x + v_unit.x * limit_dis, nearest_vertex.y + v_unit.y * limit_dis};
                proper_pos[i][j] = new_c;
            }
        }
    }
}

void init()
{
    proper_pos = vector<vector<Vertex>>(Map::width + 2, vector<Vertex>(Map::height + 2));
    pre = vector<vector<Pos>>(Map::width + 2, vector<Pos>(Map::height + 2));

    for (int i = 1; i <= Map::width; ++i)
    {
        for (int j = 1; j <= Map::height; ++j)
        {
            proper_pos[i][j] = get_center(i, j);
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        get_proper_pos();
    }
}

vector<Vertex> get_ori_path()
{
    double limit_dis = have_good ? ConVar::robot_radius_goods * 2 : ConVar::robot_radius * 2;
    auto start_pos = current_pos(start);
    auto target_pos = current_pos(target);

    if (start_pos.index_x == target_pos.index_x and start_pos.index_y == target_pos.index_y)
    {
        return {start, target};
    }

    vector<Vertex> ori_path;

    priority_queue<pair<Pos, int>, vector<pair<Pos, int>>, decltype(cmp_pos)> que(cmp_pos);
    queue<Pos> que_bak;

    pre[start_pos.index_x][start_pos.index_y].index_x = -2;
    que.push({start_pos, 0});
    que_bak.push(start_pos);

    const auto &target_pre = pre[target_pos.index_x][target_pos.index_y];
    while (not que.empty())
    {
        if (target_pre.index_x != -1) break;

        const pair<Pos, int> now_pos = que.top();
        que.pop();

        Vertex now_center = get_center(now_pos.first.index_x, now_pos.first.index_y);

        for (int i = 0; i < 4; ++i)
        {
            int nx = now_pos.first.index_x + dirs[i].first;
            int ny = now_pos.first.index_y + dirs[i].second;

            if (meta.map[nx][ny] == '#') continue;

            auto &npre = pre[nx][ny];
            if (npre.index_x != -1) continue;

            bool is_stop = false;
            Vertex ncenter = get_center(nx, ny);
            for (const auto &line : trans_map::stop_line)
            {
                if (Segment::is_cross_2(line, Segment {now_center, ncenter}))
                {
                    is_stop = true;
                    break;
                }
            }
            if (is_stop) continue;

            for (const auto &line : trans_map::danger_line)
            {
                if (not Segment::is_cross_2(line, Segment {now_center, ncenter})) continue;

                if (line.length() <= limit_dis)
                {
                    is_stop = true;
                    break;
                }
            }
            if (is_stop) continue;

            // 偏移起点和终点之外所有点
            Pos npos = {nx, ny, proper_pos[nx][ny]};

            npre = now_pos.first;
            que.push({npos, now_pos.second + 1});
            que_bak.push(npos);
        }
    }

    if (target_pre.index_x == -1)
    {
        ori_path = {};
    }
    else
    {
        Pos now_pos = target_pos;
        do
        {
            ori_path.push_back(now_pos.pos);
            now_pos = pre[now_pos.index_x][now_pos.index_y];
        } while (now_pos.index_x != -2);

        reverse(ori_path.begin(), ori_path.end());
    }

    while (not que_bak.empty())
    {
        Pos now_pos = que_bak.front();
        que_bak.pop();
        pre[now_pos.index_x][now_pos.index_y].index_x = -1;
    }

    return ori_path;
}

vector<Vertex> get_smooth_path(const vector<Vertex> &ori_path)
{
    if (ori_path.empty()) return {};

    vector<Vertex> smooth_path;

    double limit_dis = have_good ? ConVar::robot_radius_goods : ConVar::robot_radius;
    int start_index = 0;
    smooth_path.push_back(ori_path[start_index]);

    // size 一定大于1
    while (start_index < ori_path.size() - 1)
    {
        int end_index = 0;
        for (int i = start_index + 1; i < ori_path.size(); ++i)
        {
            bool is_valid = true;
            Segment tmp = {ori_path[start_index], ori_path[i]};
            // TODO 复杂度高
            for (const auto &poly : trans_map::polys)
            {
                const auto &points = poly.points;
                for (int j = 0; j < points.size(); ++j)
                {
                    if (Segment::distance(tmp, {points[j], points[(j + 1) % points.size()]}) < limit_dis)
                    {
                        is_valid = false;
                        break;
                    }
                }
                if (not is_valid) break;
            }

            if (i == ori_path.size() - 1)
            {
                end_index = i;
            }
            else
            {
                if (is_valid) continue;
                if (start_index + 1 == i)
                {
                    // TODO 此时应该让这条线段偏移
                    end_index = i;
                }
                else
                {
                    end_index = i - 1;
                }
            }
            smooth_path.push_back(ori_path[end_index]);
            start_index = end_index;
        }
    }

    return smooth_path;
}


// vector<Vertex> smooth_path(vector<Vertex> path);
// wrap 函数
vector<Vertex> find_path(const Vertex &_start, const Vertex &_target, bool _have_good)
{
    start = _start;
    target = _target;
    have_good = _have_good;
    const auto &ori_path = get_ori_path();
    const auto &smooth_path = get_smooth_path(ori_path);
    return smooth_path;
}


vector<Vertex>
find_shelter_path(const vector<Vertex> &sub_path, const vector<vector<Vertex>> &pri_path, bool have_good)
{
    // 不在路径上继续走
    // 在路径上 尝试3个格子内走出路径
    // 走不出 就顺着对方路径走

    Vertex start = sub_path[0];

    bool is_in_path = false;
    for (const auto &tmp_path : pri_path)
    {

        for (int i = 0; i < tmp_path.size() and not is_in_path; ++i)
        {
            if (dis_point_to_segment(start, {tmp_path[i], tmp_path[(i + 1) % tmp_path.size()]}) <= 1.2)
            {
                is_in_path = true;
                break;
            }
        }
        if (not is_in_path) continue;

        Pos now_pos = current_pos(start);
        // 在路径上尝试走出去

        for (int i = max(1, now_pos.index_x - 5); i <= min(Map::width, now_pos.index_x + 5); ++i)
        {
            for (int j = max(1, now_pos.index_y - 5); j <= min(Map::height, now_pos.index_y + 5); ++j)
            {
                if (meta.map[i][j] == '#') continue;
                bool is_run = true;
                Vertex center = get_center(i, j);
                for (const auto &_tmp_path : pri_path)
                {
                    for (int k = 0; k < _tmp_path.size(); ++k)
                    {
                        if (dis_point_to_segment(
                                center, {_tmp_path[k], _tmp_path[(k + 1) % _tmp_path.size()]})
                            <= 1.2)
                        {
                            is_run = false;
                            break;
                        }
                    }
                    if (not is_run) break;
                }

                if (is_run)
                {
                    auto result = find_path(start, center, have_good);
                    if (not result.empty()) return result;
                }
            }
        }
        return find_path(start, tmp_path.back(), have_good);
    }
    return find_path(start, start, have_good);
}
};
#endif