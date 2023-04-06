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


vector<pair<int, int>> dirs = {
    { 0,  1},
    { 1,  0},
    { 0, -1},
    {-1,  0},
};

// 获得正方形重心
Vertex get_center(int i, int j) { return {(i - 1) * 0.5 + 0.25, (j - 1) * 0.5 + 0.25}; }

struct Find_path {

    static vector<vector<Pos>> proper_pos;
    static vector<vector<Pos>> proper_pos_good;

    const Vertex start;
    const Vertex target;
    const bool have_good;

    vector<Vertex> ori_path;
    vector<Vertex> proper_path;
    vector<int> fix_flag;
    vector<Vertex> fix_path;
    vector<Vertex> smooth_path;
    vector<vector<Pos>> pre = vector<vector<Pos>>(Map::width + 2, vector<Pos>(Map::height + 2));
    vector<vector<int>> step = vector<vector<int>>(Map::width + 2, vector<int>(Map::height + 2, -1));

    static void init()
    {
        for (int i = 1; i <= Map::width; ++i)
        {
            for (int j = 1; j <= Map::height; ++j)
            { }
        }
    }


    void get_ori_path()
    {
        double limit_dis = have_good ? ConVar::robot_radius_goods * 2 : ConVar::robot_radius * 2;
        auto start_pos = current_pos(start);
        auto target_pos = current_pos(target);

        if (start_pos.index_x == target_pos.index_x and start_pos.index_y == target_pos.index_y)
        {
            ori_path.push_back(start);
            ori_path.push_back(target);
            fix_flag = vector<int>(ori_path.size(), 0);
            fix_flag[0] = 1;
            fix_flag[ori_path.size() - 1] = 1;
            return;
        }

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


        if (pre[target_pos.index_x][target_pos.index_y].index_x == -1)
        {
            ori_path = {};
            return;
        }

        Pos now_pos = target_pos;
        do
        {
            ori_path.push_back(now_pos.pos);
            now_pos = pre[now_pos.index_x][now_pos.index_y];
        } while (now_pos.index_x != -2);

        reverse(ori_path.begin(), ori_path.end());

        if (!ori_path.empty())
        {
            fix_flag = vector<int>(ori_path.size(), 0);
            fix_flag[0] = 1;
            fix_flag[ori_path.size() - 1] = 1;
        }
    }

    void get_ori_path_pri()
    {
        vector<pair<int, int>> dirs = {
            { 0,  1},
            { 1,  0},
            { 0, -1},
            {-1,  0},
        };

        double limit_dis = have_good ? ConVar::robot_radius_goods * 2 : ConVar::robot_radius * 2;
        auto start_pos = current_pos(start);
        auto target_pos = current_pos(target);

        if (start_pos.index_x == target_pos.index_x and start_pos.index_y == target_pos.index_y)
        {
            ori_path.push_back(start);
            ori_path.push_back(target);
            fix_flag = vector<int>(ori_path.size(), 0);
            fix_flag[0] = 1;
            fix_flag[ori_path.size() - 1] = 1;
            return;
        }

        // 优先队列
        auto cmp_pos = [&](const Pos &a, const Pos &b) -> bool {
            if (step[a.index_x][a.index_y] == step[b.index_x][b.index_y])
                return trans_map::nearest_obstacle[a.index_x][a.index_y]
                    < trans_map::nearest_obstacle[b.index_x][b.index_y];
            else
                return step[a.index_x][a.index_y] > step[b.index_x][b.index_y];
        };

        priority_queue<Pos, vector<Pos>, decltype(cmp_pos)> que(cmp_pos);
        que.push(start_pos);
        pre[start_pos.index_x][start_pos.index_y].index_x = -2;
        step[start_pos.index_x][start_pos.index_y] = 0;
        int now_step = 0;
        while (not que.empty())
        {
            if (pre[target_pos.index_x][target_pos.index_y].index_x != -1) break;

            const auto now_pos = que.top();
            que.pop();

            // cerr << "now_pos: " << now_pos.index_x << " " << now_pos.index_y << endl;
            // cerr << "now_step: " << now_step << endl;
            // cerr << "step: " << step[now_pos.index_x][now_pos.index_y] << endl;
            // cerr << "distance: " << trans_map::nearest_obstacle[now_pos.index_x][now_pos.index_y] <<
            // endl;

            if (step[now_pos.index_x][now_pos.index_y] < now_step)
            {
                throw "error";
            }
            else
            {
                now_step = step[now_pos.index_x][now_pos.index_y];
            }

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

                step[nx][ny] = step[now_pos.index_x][now_pos.index_y] + 1;
                que.push({nx, ny, ncenter});
                pre[nx][ny] = now_pos;
                // cerr << "push: " << nx << " " << ny << endl;
            }
        }


        if (pre[target_pos.index_x][target_pos.index_y].index_x == -1)
        {
            ori_path = {};
            return;
        }

        Pos now_pos = target_pos;
        do
        {
            ori_path.push_back(now_pos.pos);
            now_pos = pre[now_pos.index_x][now_pos.index_y];
        } while (now_pos.index_x != -2);

        reverse(ori_path.begin(), ori_path.end());

        if (!ori_path.empty())
        {
            fix_flag = vector<int>(ori_path.size(), 0);
            fix_flag[0] = 1;
            fix_flag[ori_path.size() - 1] = 1;
        }
    }

    // 将点偏移到正确位置
    void get_proper_path()
    {
        proper_path = ori_path;
        double limit_path = have_good ? ConVar::robot_radius_goods : ConVar::robot_radius;
        limit_path += 0.05;
        for (int i = 0; i < proper_path.size(); ++i)
        {
            if (fix_flag[i]) continue;
            Vertex nearest_vertex;
            double nearest_dis = 1e9;
            for (const auto poly : trans_map::polys)
            {
                const auto points = poly.points;
                for (int j = 0; j < points.size(); ++j)
                {
                    Segment line = {points[j], points[(j + 1) % points.size()]};
                    double tmp_dis = dis_point_to_segment(proper_path[i], line);
                    if (tmp_dis <= nearest_dis)
                    {
                        nearest_dis = tmp_dis;
                        nearest_vertex = point_point_to_segment(proper_path[i], line);
                    }
                }
            }

            if (nearest_dis < limit_path)
            {
                Vec2 v = {nearest_vertex, proper_path[i]};
                // c 沿着 v 方向移动 limit_path
                // XXX v 理论上应该不可能为 0
                Vec2 v_unit = {v.x / v.length(), v.y / v.length()};
                Vertex new_c
                    = {nearest_vertex.x + v_unit.x * limit_path, nearest_vertex.y + v_unit.y * limit_path};
                proper_path[i] = new_c;
            }
        }
    }

    void get_fix_path()
    {
        fix_path = proper_path;
        fix_flag = vector<int>(fix_path.size(), 0);
        for (int i = 0; i < int(fix_path.size()) - 1; ++i)
        {
            int j = i + 1;
            if (fix_flag[j]) continue;
            for (const auto line : trans_map::danger_line)
            {
                if (Segment::is_cross_2(line, Segment {fix_path[i], fix_path[j]}))
                {
                    fix_path[j] = Segment::get_mid(line);
                    fix_flag[j] = 1;
                    break;
                }
            }
        }
    }

    void get_smooth_path(const vector<Vertex> &_path)
    {
        double limit_dis = have_good ? ConVar::robot_radius_goods : ConVar::robot_radius;
        int start_index = 0;
        const auto &tmp_path = _path;
        if (tmp_path.empty()) return;
        smooth_path.push_back(tmp_path[start_index]);
        while (start_index < int(tmp_path.size()) - 1)
        {
            int end_index = 0;
            for (int i = start_index + 1; i < tmp_path.size(); ++i)
            {
                bool is_valid = true;
                Segment tmp = {tmp_path[start_index], tmp_path[i]};
                for (const auto poly : trans_map::polys)
                {
                    const auto points = poly.points;
                    for (int j = 0; j < points.size(); ++j)
                    {
                        if (Segment::distance(tmp, {points[j], points[(j + 1) % points.size()]})
                            < limit_dis)
                        {
                            is_valid = false;
                            break;
                        }
                    }
                    if (not is_valid) break;
                }

                if (i == int(tmp_path.size()) - 1)
                {
                    end_index = i;
                }
                else
                {
                    if (is_valid)
                    {
                        if (fix_flag[i] == 1)
                        {
                            end_index = i;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        if (i == 1)
                        {
                            end_index = i;
                            bool is_valid_2 = true;
                        }
                        else
                        {
                            end_index = i - 1;
                        }
                    }
                }
                smooth_path.push_back(tmp_path[end_index]);
                start_index = end_index;
            }
        }
    }

    void get_path()
    {
        get_ori_path();
        if (ori_path.size() >= 3)
        {
            if (ori_path[0] == ori_path[1])
            {
                throw "start == target";
            }
        }

        get_smooth_path(ori_path);
        if (smooth_path.size() >= 3)
        {
            if (smooth_path[0] == smooth_path[1])
            {
                throw "start == target";
            }
        }
    }
    void get_path_pri()
    {
        get_ori_path_pri();
        if (ori_path.size() >= 3)
        {
            if (ori_path[0] == ori_path[1])
            {
                throw "start == target";
            }
        }
        get_proper_path();
        if (proper_path.size() >= 3)
        {
            if (proper_path[0] == proper_path[1])
            {
                throw "start == target";
            }
        }

        get_smooth_path(proper_path);
        if (smooth_path.size() >= 3)
        {
            if (smooth_path[0] == smooth_path[1])
            {
                throw "start == target";
            }
        }
    }

    Find_path(const Vertex &start, const Vertex &target, bool have_good) :
        start(start), target(target), have_good(have_good)
    { }
};

// vector<Vertex> smooth_path(vector<Vertex> path);
// wrap 函数
vector<Vertex> find_path(const Vertex &start, const Vertex &target, bool have_good)
{
    Find_path find_path(start, target, have_good);
    find_path.get_path_pri();
    return find_path.smooth_path;
}

// vector<Vertex> find_path_pri(const Vertex &start, const Vertex &target, bool have_good)
// {
//     Find_path find_path(start, target, have_good);
//     find_path.get_path_pri();
//     return find_path.smooth_path;
// }

// // 检查line是否跟多边形相交
// bool check_cross(Segment line)
// {
//     for (auto &poly : trans_map::polys)
//         for (int i = 1; i < poly.points.size(); i++)
//             if (Segment::is_cross(line, {poly.points[i - 1], poly.points[i]})) return true;

//     return false;
// }
// bool check_valid(Segment line)
// {
//     if(std::hypot(line.a.x - line.b.x, line.a.y - line.b.y) < EPS) return true;
//     auto start = line.a;
//     auto target = line.b;
//     bool valid = true;
//     if (check_cross({start, target})) valid = false;
//     if (valid)
//     {
//         // 检测线段沿着垂直方向移动后是否可以直接连线
//         Vec2 rotate_dir = Vec2 {start, target}.rotate(M_PI / 2) * 10;
//         rotate_dir = rotate_dir / rotate_dir.length() * 0.6;
//         Vertex start_1 = {start.x + rotate_dir.x, start.y + rotate_dir.y};
//         Vertex target_1 = {target.x + rotate_dir.x, target.y + rotate_dir.y};
//         valid = !check_cross({start_1, target_1});
//     }
//     if (valid)
//     {
//         Vec2 rotate_dir = Vec2 {start, target}.rotate(-M_PI / 2) * 10;
//         rotate_dir = rotate_dir / rotate_dir.length() * 0.6;
//         Vertex start_1 = {start.x + rotate_dir.x, start.y + rotate_dir.y};
//         Vertex target_1 = {target.x + rotate_dir.x, target.y + rotate_dir.y};
//         valid = !check_cross({start_1, target_1});
//     }
//     return valid;
// }
// vector<Vertex> smooth_path(vector<Vertex> path)
// {
//     for (int i = 1; i < path.size(); i++)
//     {
//         if (path[i] == path[i - 1])
//         {
//             path.erase(path.begin() + i);
//             i--;
//         }
//     }
//     if (path.size() < 3) return path;
//     cerr << "[info][smooth_path]" << endl;
//     cerr << "before_smooth = [";
//     for (auto &v : path)
//         cerr << v << ",";
//     cerr << "]" << endl;
//     vector<Vertex> smooth_path;
//     smooth_path.push_back(path[0]);
//     smooth_path.push_back(path[1]);
//     for (int i = 2; i < path.size(); i++)
//     {
//         auto start = smooth_path[smooth_path.size() - 2];
//         auto target = path[i];
//         // 检查是否可以直接连线
//         bool replace = check_valid({start, target});
//         if (replace) smooth_path.pop_back();

//         smooth_path.push_back(target);
//     }


//     // 用两条直线的交点代替中间的两个点
//     path = smooth_path;
//     smooth_path.clear();
//     if (path.size() < 5) return path;
//     smooth_path.push_back(path[0]);
//     smooth_path.push_back(path[1]);
//     smooth_path.push_back(path[2]);
//     for (int i = 3; i < path.size(); i++)
//     {
//         smooth_path.push_back(path[i]);
//         Segment line1 {smooth_path[smooth_path.size() - 4], smooth_path[smooth_path.size() - 3]};
//         Segment line2 {smooth_path[smooth_path.size() - 2], smooth_path[smooth_path.size() - 1]};
//         // 标准法求两条直线的交点
//         Vertex cross_point;
//         double a1 = line1.b.y - line1.a.y;
//         double b1 = line1.a.x - line1.b.x;
//         double c1 = line1.b.x * line1.a.y - line1.a.x * line1.b.y;
//         double a2 = line2.b.y - line2.a.y;
//         double b2 = line2.a.x - line2.b.x;
//         double c2 = line2.b.x * line2.a.y - line2.a.x * line2.b.y;
//         double d = a1 * b2 - a2 * b1;
//         // 检查是否平行
//         if (fabs(d) < EPS) continue;

//         cross_point.x = (b1 * c2 - b2 * c1) / d;
//         cross_point.y = (a2 * c1 - a1 * c2) / d;

//         if (check_valid({line1.a, cross_point}) and check_valid({cross_point, line2.b}))
//         {
//             smooth_path.pop_back();
//             smooth_path.pop_back();
//             smooth_path.pop_back();
//             smooth_path.push_back(cross_point);
//             smooth_path.push_back(path[i]);
//         }
//     }

//     cerr << "after_smooth = [";
//     for (auto &v : smooth_path)
//         cerr << v << ",";
//     cerr << "]" << endl;
//     cerr << "from " << path.size() << " to " << smooth_path.size() << endl;
//     return smooth_path;
// }

vector<Vertex>
find_shelter_path(const Vertex &start, const vector<vector<Vertex>> &pri_path, bool have_good)
{
    // 不在路径上不动
    // 在路径上 尝试3个格子内走出路径
    // 走不出 就顺着对方路径走

    bool is_in_path = false;
    for (auto tmp_path : pri_path)
    {
        for (int i = 0; i < tmp_path.size() and not is_in_path; ++i)
        {
            if (dis_point_to_segment(start, {tmp_path[i], tmp_path[(i + 1) % tmp_path.size()]}) <= 1.5)
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
                for (auto _tmp_path : pri_path)
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