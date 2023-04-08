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

int cnt_path = 0;
int pre_cnt_path = 0;

int cnt_find_que = 0;
int pre_cnt_find_que = 0;
// vector<vector<Vertex>> proper_pos;
vector<vector<Pos>> pre = vector<vector<Pos>>(Map::width + 2, vector<Pos>(Map::height + 2));

Vertex start;
Vertex target;
bool have_good;

vector<pair<int, int>> dirs = {
    { 0,  1},
    { 1,  0},
    { 0, -1},
    {-1,  0},
};

bool _is_valid(Segment line)
{
    ++cnt_path;
    double limit_dis = have_good ? ConVar::robot_radius_goods * 2 : ConVar::robot_radius * 2;
    for (const auto &poly : trans_map::polys)
    {
        const auto &points = poly.points;
        for (int j = 0; j < points.size(); ++j)
        {
            if (Segment::distance(line, {points[j], points[(j + 1) % points.size()]}) < limit_dis)
            {
                return false;
            }
        }
    }
    return true;
}

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


// 获得正方形重心
inline Vertex get_center(int i, int j) { return {(i - 1) * 0.5 + 0.25, (j - 1) * 0.5 + 0.25}; }


// 优先队列排序函数
auto cmp_pos = [](const pair<Pos, int> &a, const pair<Pos, int> &b) -> bool {
    return a.second > b.second
        or (a.second == b.second
            and trans_map::nearest_obstacle[a.first.index_x][a.first.index_y]
                < trans_map::nearest_obstacle[b.first.index_x][b.first.index_y]);
};


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
        cnt_find_que++;
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
            if (start_pos.index_x != now_pos.first.index_x or start_pos.index_y != now_pos.first.index_y)
            {
                for (const auto &line : trans_map::stop_line)
                {
                    {
                        if (Segment::is_cross(line, Segment {now_center, ncenter}))
                        {
                            is_stop = true;
                            break;
                        }
                    }
                }
            }
            if (is_stop and not(nx == target_pos.index_x and ny == target_pos.index_y)) continue;

            bool is_two_squre = false;
            for (const auto &line : trans_map::danger_line)
            {
                if (not Segment::is_cross_2(line, Segment {now_center, ncenter})) continue;

                is_two_squre = true;
                ncenter = {(line.a.x + line.b.x) / 2, (line.a.y + line.b.y) / 2};
                if (have_good)
                {
                    is_stop = true;
                    break;
                }
            }
            if (is_stop) continue;
            // 四个方向上的点在地图上没有#
            if (not(nx == target_pos.index_x and ny == target_pos.index_y) and not is_two_squre
                and (meta.map[nx][ny + 1] == '#' or meta.map[nx][ny - 1] == '#'
                    or meta.map[nx + 1][ny] == '#' or meta.map[nx - 1][ny] == '#'))
                continue;


            Pos npos = {nx, ny, ncenter};

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
            Segment tmp = {ori_path[start_index], ori_path[i]};
            // TODO 复杂度高

            bool is_valid = _is_valid(tmp);

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

    // xxx 需要跟nav_nagivatorp停留在原地距离配合
    while (smooth_path.size() > 2 and Vertex::distance(smooth_path[0], smooth_path[1]) < 0.1)
    {
        smooth_path.erase(smooth_path.begin());
    }

    return smooth_path;
}


vector<Vertex> smooth_path_again(vector<Vertex> path)
{
    if (path.size() < 5) return path;
    auto ori_path = path;
    // 用两条直线的交点代替中间的两个点
    size_t window_size = 30;

    for (int i = 1; i < path.size(); i++)
    {
        // i-1->i 与 j-1->j能否合法相交
        Segment line1 = {path[i - 1], path[i]};
        for (int j = i + 2; j < min(i + 30, int(path.size())); j++)
        {
            Segment line2 = {path[j - 1], path[j]};
            // 标准法求两条直线的交点
            Vertex cross_point;
            double a1 = line1.b.y - line1.a.y;
            double b1 = line1.a.x - line1.b.x;
            double c1 = line1.b.x * line1.a.y - line1.a.x * line1.b.y;
            double a2 = line2.b.y - line2.a.y;
            double b2 = line2.a.x - line2.b.x;
            double c2 = line2.b.x * line2.a.y - line2.a.x * line2.b.y;
            double d = a1 * b2 - a2 * b1;
            // 检查是否平行
            if (fabs(d) < EPS) continue;

            cross_point.x = (b1 * c2 - b2 * c1) / d;
            cross_point.y = (a2 * c1 - a1 * c2) / d;

            if (_is_valid({line1.a, cross_point}) and _is_valid({cross_point, line2.b}))
            {
                if (_USE_LOG_)
                {
                    cerr << "[debug][smooth_path_again] CONGRACTULATIONS! " << i << " " << j << endl;
                }
                // 删除掉[i, j-1]的点
                path[i] = cross_point;
                for (size_t k = i + 1; k < j; k++)
                    path.erase(path.begin() + i + 1);
                break;
            }
        }
    }
    if (_USE_LOG_)
    {
        if (path.size() < ori_path.size())
        {
            cerr << "smooth_path_again: " << ori_path.size() << " -> " << path.size() << endl;
            cerr << "before_smooth = [";

            for (auto &v : ori_path)
                cerr << v << ",";
            cerr << "]" << endl;
            cerr << "after_smooth = [";
            for (auto &v : path)
                cerr << v << ",";
            cerr << "]" << endl;
        }
    }

    return path;
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
    return smooth_path_again(smooth_path);
}

vector<Vertex>
find_shelter_path(const vector<Vertex> &sub_path, const vector<vector<Vertex>> &pri_path, bool have_good)
{
    // 不在路径上继续走
    // 在路径上 尝试3个格子内走出路径
    // 走不出 就顺着对方路径走

    Vertex start = sub_path[0];
    double limit_dis = have_good ? ConVar::robot_radius_goods * 2 : ConVar::robot_radius * 2;

    bool is_in_path = false;
    for (const auto &tmp_path : pri_path)
    {
        for (int i = 0; i < tmp_path.size() and not is_in_path; ++i)
        {
            if (dis_point_to_segment(start, {tmp_path[i], tmp_path[(i + 1) % tmp_path.size()]})
                <= limit_dis)
            {
                is_in_path = true;
                break;
            }
        }
        if (is_in_path) break;
    }
    if (not is_in_path) return sub_path;

    Pos now_pos = current_pos(start);

    auto start_pos = current_pos(start);
    Vertex target = {-1, -1};

    priority_queue<pair<Pos, int>, vector<pair<Pos, int>>, decltype(cmp_pos)> que(cmp_pos);
    queue<Pos> que_bak;

    pre[start_pos.index_x][start_pos.index_y].index_x = -2;
    que.push({start_pos, 0});
    que_bak.push(start_pos);

    while (not que.empty())
    {
        cnt_find_que++;
        const pair<Pos, int> now_pos = que.top();
        que.pop();

        Vertex now_center = get_center(now_pos.first.index_x, now_pos.first.index_y);

        // 走得到 这个点距离所有对方路径的前3m的线段 距离大于1.2m
        // 避免迎头撞上对方
        bool is_run = true;
        for (const auto &tmp_path : pri_path)
        {
            double left_dis = 3;
            for (int k = 0; k < tmp_path.size(); ++k)
            {
                // 过长截取
                navmesh::Vec2 vec_tmp {tmp_path[k], tmp_path[(k + 1) % tmp_path.size()]};
                navmesh::Segment tmp_seg;
                if (vec_tmp.length() >= left_dis)
                {
                    // XXX 万一除0
                    tmp_seg = {
                        sub_path[k],
                        {sub_path[k].x + vec_tmp.x * left_dis / vec_tmp.length(),
                             sub_path[k].y + vec_tmp.y * left_dis / vec_tmp.length()}
                    };
                }
                else
                {
                    tmp_seg = {sub_path[k], sub_path[(k + 1) % sub_path.size()]};
                }
                if (dis_point_to_segment(now_center, tmp_seg) <= 1.2)
                {
                    is_run = false;
                    break;
                }
                left_dis -= vec_tmp.length();
                if (left_dis <= 1e-6) break;
            }
            if (not is_run) break;
        }

        if (is_run)
        {
            target = now_center;
            if (_USE_LOG_)
            {
                cerr << "[info][find_shelter_path] robot_id " << pri_path.size() + 1
                     << " find shelter point." << target << endl;
            }
            break;
        }

        for (int i = 0; i < 4; ++i)
        {
            int nx = now_pos.first.index_x + dirs[i].first;
            int ny = now_pos.first.index_y + dirs[i].second;

            if (meta.map[nx][ny] == '#') continue;

            auto &npre = pre[nx][ny];
            if (npre.index_x != -1) continue;

            bool is_stop = false;
            Vertex ncenter = get_center(nx, ny);
            if (start_pos.index_x != now_pos.first.index_x or start_pos.index_y != now_pos.first.index_y)
            {
                for (const auto &line : trans_map::stop_line)
                {
                    if (Segment::is_cross(line, Segment {now_center, ncenter}))
                    {
                        is_stop = true;
                        break;
                    }
                }
            }

            if (is_stop) continue;

            bool is_two_squre = false;
            for (const auto &line : trans_map::danger_line)
            {
                if (not Segment::is_cross_2(line, Segment {now_center, ncenter})) continue;

                is_two_squre = true;
                ncenter = {(line.a.x + line.b.x) / 2, (line.a.y + line.b.y) / 2};

                if (have_good)
                {
                    is_stop = true;
                    break;
                }
            }
            if (is_stop) continue;
            // 四个方向上的点在地图上没有#
            if (not is_two_squre
                and (meta.map[nx][ny + 1] == '#' or meta.map[nx][ny - 1] == '#'
                    or meta.map[nx + 1][ny] == '#' or meta.map[nx - 1][ny] == '#'))
                continue;

            if (now_pos.second == 6) continue;

            // 偏移起点和终点之外所有点
            Pos npos = {nx, ny, ncenter};

            npre = now_pos.first;
            que.push({npos, now_pos.second + 1});
            que_bak.push(npos);
        }
    }

    while (not que_bak.empty())
    {
        Pos now_pos = que_bak.front();
        que_bak.pop();
        pre[now_pos.index_x][now_pos.index_y].index_x = -1;
    }

    if (target.x != -1)
    {
        return find_path(start, target, have_good);
    }
    else
    {
        double nearest_dis = 1e9;
        int nearest_index = 0;
        for (int i = 0; i < pri_path.size(); ++i)
        {
            if (pri_path[i].empty()) continue;
            double tmp_dis = Vertex::distance(sub_path[0], pri_path[i][0]);
            if (tmp_dis < nearest_dis)
            {
                nearest_dis = tmp_dis;
                nearest_index = i;
            }
        }
        return find_path(start, pri_path[nearest_index].back(), true);
    }
}

// 找到最近的工作站,limit步数内没有找到则返回空
std::vector<Vertex> find_nearest_workshop(const Vertex &start)
{
    int step_limit = 10;    // 最多搜索20步
    double limit_dis = ConVar::robot_radius * 2;
    limit_dis *= 1.1;
    Vertex target {-1, -1};
    /* BFS寻找躲避点*/
    {
        priority_queue<pair<Pos, int>, vector<pair<Pos, int>>, decltype(cmp_pos)> que(cmp_pos);
        queue<Pos> que_bak;
        auto start_pos = current_pos(start);
        pre[start_pos.index_x][start_pos.index_y].index_x = -1;
        que.push({start_pos, 0});
        que_bak.push(start_pos);

        pair<Pos, int> now_pos {};
        while (not que.empty())
        {
            cnt_find_que++;
            now_pos = que.top();
            que.pop();
            // check valid
            if (now_pos.second > step_limit) break;

            // 找到了工作站
            if (meta.map[now_pos.first.index_x][now_pos.first.index_y] >= '0'
                and meta.map[now_pos.first.index_x][now_pos.first.index_y] <= '9')
            {
                target = now_pos.first.pos;
                break;
            }
            /*next*/
            Vertex now_center = get_center(now_pos.first.index_x, now_pos.first.index_y);
            for (int i = 0; i < 4; ++i)
            {

                int nx = now_pos.first.index_x + dirs.at(i).first;
                int ny = now_pos.first.index_y + dirs.at(i).second;

                if (meta.map.at(nx).at(ny) == '#') continue;
                auto &npre = pre.at(nx).at(ny);
                if (npre.index_x != -1) continue;

                bool is_stop = false;
                Vertex ncenter = get_center(nx, ny);
                if (start_pos.index_x != now_pos.first.index_x
                    or start_pos.index_y != now_pos.first.index_y)
                {
                    for (const auto &line : trans_map::stop_line)
                    {
                        if (Segment::is_cross(line, Segment {now_center, ncenter}))
                        {
                            is_stop = true;
                            break;
                        }
                    }
                }
                if (is_stop) continue;

                bool is_two_squre = false;
                for (const auto &line : trans_map::danger_line)
                {
                    if (not Segment::is_cross_2(line, Segment {now_center, ncenter})) continue;

                    is_two_squre = true;
                    ncenter = {(line.a.x + line.b.x) / 2, (line.a.y + line.b.y) / 2};

                    if (have_good)
                    {
                        is_stop = true;
                        break;
                    }
                }
                if (is_stop) continue;
                // 四个方向上的点在地图上没有#
                if (not is_two_squre
                    and not(meta.map[now_pos.first.index_x][now_pos.first.index_y] >= '0'
                        and meta.map[now_pos.first.index_x][now_pos.first.index_y] <= '9')
                    and (meta.map[nx][ny + 1] == '#' or meta.map[nx][ny - 1] == '#'
                        or meta.map[nx + 1][ny] == '#' or meta.map[nx - 1][ny] == '#'))
                    continue;

                // 偏移起点和终点之外所有点
                Pos npos = {nx, ny, ncenter};

                npre = now_pos.first;

                que.push({npos, now_pos.second + 1});
                que_bak.push(npos);
            }
        }

        // 释放内存
        while (not que_bak.empty())
        {
            Pos now_pos = que_bak.front();
            que_bak.pop();
            pre[now_pos.index_x][now_pos.index_y].index_x = -1;
        }
    }
    if (target.x != -1)
        return find_path(start, target, false);
    else
        return {};
}
};
#endif