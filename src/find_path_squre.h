#ifndef __FIND_PATH_SQURE_H__
#define __FIND_PATH_SQURE_H__
#include "const.h"
#include "nav_model.h"
#include "trans_map.h"
#include <algorithm>
#include <optional>
#include <queue>
#include <vector>
namespace find_path_square
{
using namespace std;
using namespace navmesh;


struct Pos {
    int index_x = -1;
    int index_y = -1;
    Vertex pos;
};

struct find_path {

    const Vertex start;
    const Vertex target;
    const bool have_good;
    vector<Vertex> ori_path;
    vector<Vertex> proper_path;
    vector<int> fix_flag;
    vector<Vertex> fix_path;
    vector<Vertex> smooth_path;

    find_path(const Vertex &start, const Vertex &target, bool have_good) :
        start(start), target(target), have_good(have_good)
    { }

    static Pos current_pos(const Vertex &v)
    {

        for (int i = 2 * int(v.x); i <= 2 * int(v.x) + 2; ++i)
        {
            for (int j = 2 * int(v.y); j <= 2 * int(v.y) + 2; ++j)
            {
                vector<Vertex> vs {
                    {(i - 1) * 0.5,       j * 0.5},
                    {(i - 1) * 0.5, (j - 1) * 0.5},
                    {      i * 0.5, (j - 1) * 0.5},
                    {      i * 0.5,       j * 0.5},
                };

                if (trans_map::is_in_polygon(vs, v))
                {
                    return {i, j, v};
                }
            }
        }
        throw "you are wrong";
    }

    // 获得正方形重心
    static const Vertex get_center(int i, int j) { return {(i - 1) * 0.5 + 0.25, (j - 1) * 0.5 + 0.25}; }


    void get_ori_path()
    {

        vector<pair<int, int>> dirs = {
            { 0,  1},
            { 1,  0},
            { 0, -1},
            {-1,  0},
        };

        vector<vector<Pos>> pre(Map::width + 2, vector<Pos>(Map::height + 2));

        double limit_dis = have_good ? ConVar::robot_radius_goods * 2 : ConVar::robot_radius * 2;
        auto start_pos = current_pos(start);
        auto target_pos = current_pos(target);
        queue<Pos> que;
        que.push(start_pos);
        pre[start_pos.index_x][start_pos.index_y].index_x = -2;
        while (not que.empty())
        {
            if (pre[target_pos.index_x][target_pos.index_y].index_x != -1) break;

            const auto now_pos = que.front();
            que.pop();

            Vertex now_center = get_center(now_pos.index_x, now_pos.index_y);

            for (int i = 0; i < 4; ++i)
            {
                int nx = now_pos.index_x + dirs[i].first;
                int ny = now_pos.index_y + dirs[i].second;

                if (meta.map[nx][ny] == '#') continue;


                if (pre[nx][ny].index_x != -1) continue;


                bool is_stop = false;
                Vertex ncenter = get_center(nx, ny);
                for (auto line : trans_map::stop_line)
                {
                    if (Segment::is_cross_2(line, Segment {now_center, ncenter}))
                    {
                        is_stop = true;
                        break;
                    }
                }
                if (is_stop) continue;


                for (auto line : trans_map::danger_line)
                {
                    if (not Segment::is_cross_2(line, Segment {now_center, ncenter})) continue;

                    if (line.length() <= limit_dis)
                    {
                        is_stop = true;
                        break;
                    }
                }
                if (is_stop) continue;

                que.push({nx, ny, ncenter});
                pre[nx][ny] = now_pos;
            }
        }

        Pos now_pos = target_pos;

        do
        {
            ori_path.push_back(now_pos.pos);
            now_pos = pre[now_pos.index_x][now_pos.index_y];
        } while (now_pos.index_x != -2);

        reverse(ori_path.begin(), ori_path.end());
    }

    // 将点偏移到正确位置
    void get_proper_path()
    {
        double limit_path = have_good ? ConVar::robot_radius_goods : ConVar::robot_radius;
        for (int i = 0; i < ori_path.size(); ++i)
        {
            for (const auto poly : trans_map::polys)
            {
                const auto points = poly.points;
                for (int j = 0; j < points.size(); ++j)
                {
                    int k = (j + 1) % points.size();
                    if (dis_point_to_segment(ori_path[i], Segment {points[j], points[k]}) < limit_path + 0.05)
                    {
                        proper_path.push_back(Segment::get_mid(Segment {points[j], points[k]}));
                        break;
                    }
                }
            }
        }
    }

    void get_fix_path()
    {
        fix_path = ori_path;
        fix_flag = vector<int>(fix_path.size(), 0);
        for (int i = 0; i < fix_path.size() - 1; ++i)
        {
            int j = i + 1;
            for (const auto line : trans_map::danger_line)
            {
                if (Segment::is_cross_2(line, Segment {fix_path[i], fix_path[j]}))
                {
                    fix_path[j] = Segment::get_mid(line);
                    fix_flag[i] = 1;
                    break;
                }
            }
        }
    }

    vector<Vertex> get_smooth_path()
    {
        double limit_dis = have_good ? ConVar::robot_radius_goods : ConVar::robot_radius;
        int start_index = 0;
        while (start_index < fix_path.size())
        {
            int end_index = 0;
            for (int i = start_index + 1; i < fix_path.size(); ++i)
            {
                bool is_valid = true;
                for (const auto poly : trans_map::polys)
                {
                    const auto points = poly.points;
                    for (int j = 0; j < points.size(); ++j)
                    {
                        int k = (j + 1) % points.size();
                        if (Segment::distance(
                                Segment {fix_path[start_index], fix_path[i]}, {points[j], points[k]})
                            <= limit_dis)
                        {
                            is_valid = false;
                            break;
                        }
                    }
                    if (not is_valid) break;
                }

                if (is_valid and fix_flag[i])
                {
                    end_index = i;
                    break;
                }
                if (not is_valid)
                {
                    end_index = i - 1;
                    break;
                }
            }
        }
        return smooth_path;
    }

    vector<Vertex> get_path(const auto &start, const auto &target, bool have_good)
    {
        get_ori_path(start, target, have_good);
        get_fix_path(ori_path);
    }
};
};
#endif