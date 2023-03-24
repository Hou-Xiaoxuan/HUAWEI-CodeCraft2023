#ifndef __ANTICOLLISION_H__
#define __ANTICOLLISION_H__

#include "const.h"
#include "model.h"
#include "route_fool.h"
#include <iostream>
#include <optional>
#include <set>
#include <vector>

namespace anticollision
{
using namespace std;
using io::instructions;
const auto &robots = meta.robot;

Point get_spec_dir_shelter(int robot_id, int RotateDirection)
{
    const auto &robot = robots[robot_id];
    Point center;    // 圆心
    center.x = robot.loc.x + RotateDirection * ComVar::max_radius * sin(robot.dirc);
    center.y = robot.loc.y - RotateDirection * ComVar::max_radius * cos(robot.dirc);

    double shelter_angular = robot.dirc + M_PI / 2;
    Point shelter_point;
    shelter_point.x = center.x - RotateDirection * ComVar::max_radius * sin(shelter_angular);
    shelter_point.y = center.y + RotateDirection * ComVar::max_radius * cos(shelter_angular);
    return shelter_point;
}

Point get_shelter_aginst_point(int robot_id, Point point)
{
    vector<Point> tmp_shelter = {get_spec_dir_shelter(robot_id, 1), get_spec_dir_shelter(robot_id, -1)};
    return Point::distance(point, tmp_shelter[0]) > Point::distance(point, tmp_shelter[1]) ? tmp_shelter[0]
                                                                                           : tmp_shelter[1];
}

vector<int> shelter_type(ConVar::max_robot + 1, 0);
vector<Point> shelters(ConVar::max_robot + 1);
vector<int> left_shelter_cnt = vector<int>(ConVar::max_robot + 1, 0);

void cnt_down_shelter()
{
    for (int i = 1; i < shelters.size(); ++i)
    {
        if (left_shelter_cnt[i] > 0)
        {
            --left_shelter_cnt[i];
        }
        if (left_shelter_cnt[i] == 0)
        {
            shelter_type[i] = 0;
        }
        cerr << "error: robot " << i << " shelter " << shelters[i] << " left count " << left_shelter_cnt[i]
             << endl;
    }
}

void anticollision(const vector<optional<route_fool::Route>> &routes)
{
    int persisitent_flame = 7;
    int max_predict_flame = 15;

    cnt_down_shelter();

    vector<vector<Point>> predict_point(max_predict_flame + 1, vector<Point>(robots.size()));
    for (int predict_flame = 1; predict_flame <= max_predict_flame; ++predict_flame)
    {
        double predict_time = predict_flame * ComVar::flametime;
        for (int i = 1; i < robots.size(); ++i)
        {
            const auto &robot = robots[i];

            if (fabs(robot.w) < M_PI / 10)
            {
                predict_point[predict_flame][i].x = robot.loc.x + robot.v.x * predict_time;
                predict_point[predict_flame][i].y = robot.loc.y + robot.v.y * predict_time;
                continue;
            }

            double radius = robot.v.len() / robot.w;    // 半径
            Point center;                               // 圆心
            center.x = robot.loc.x - radius * sin(robot.dirc);
            center.y = robot.loc.y + radius * cos(robot.dirc);

            double predict_angular = robot.dirc + robot.w * predict_time;
            predict_point[predict_flame][i].x = center.x + radius * sin(predict_angular);
            predict_point[predict_flame][i].y = center.y - radius * cos(predict_angular);
        }
    }

    vector<vector<int>> vis = vector<vector<int>>(robots.size(), vector<int>(robots.size(), 0));
    for (int predict_flame = 1; predict_flame <= max_predict_flame; ++predict_flame)
    {
        for (int i = 1; i < robots.size(); ++i)
        {
            for (int j = i + 1; j < robots.size(); ++j)
            {
                if (vis[i][j]) continue;

                double min_distance = 2 * ConVar::robot_radius_goods + 0.1;
                if (Point::distance(predict_point[predict_flame][i], predict_point[predict_flame][j])
                    <= min_distance)
                {
                    vis[i][j] = 1;
                    Point collision_point
                        = {(predict_point[predict_flame][i].x + predict_point[predict_flame][j].x) / 2,
                            (predict_point[predict_flame][i].y + predict_point[predict_flame][j].y) / 2};

                    cerr << "info: robot " << i << " and robot " << j << " will collide in "
                         << predict_flame << " flame(s) at " << collision_point << endl;

                    if (left_shelter_cnt[i] != 0 && left_shelter_cnt[j] != 0)
                    {
                        cerr << "info: robot " << i << " and robot " << j
                             << " have already been assigned shelters." << endl;
                        cerr << "info: robot fate is decided." << endl;
                        cerr << "info: robot " << i << " shelter " << shelters[i] << " left count "
                             << left_shelter_cnt[i] << endl;
                        cerr << "info: robot " << j << " shelter " << shelters[j] << " left count "
                             << left_shelter_cnt[j] << endl;
                    }
                    else if (left_shelter_cnt[i] != 0 || left_shelter_cnt[j] != 0)
                    {
                        int shelter_id = left_shelter_cnt[i] == 0 ? i : j;
                        int fix_id = left_shelter_cnt[i] == 0 ? j : i;

                        double delta_angle = fabs(robots[i].dirc - robots[j].dirc);
                        if (delta_angle > M_PI)
                            delta_angle -= 2 * M_PI;
                        else if (delta_angle < -M_PI)
                            delta_angle += 2 * M_PI;

                        if (shelter_type[fix_id] == 2)
                        {
                            shelter_type[shelter_id] = 1;
                            shelters[shelter_id] = get_shelter_aginst_point(shelter_id, robots[fix_id].loc);
                            left_shelter_cnt[shelter_id] = 1;
                        }

                        else
                        {
                            if (fabs(delta_angle) < M_PI / 9)
                            {
                                shelter_type[shelter_id] = 2;
                                left_shelter_cnt[shelter_id] = persisitent_flame;
                            }
                            else
                            {
                                shelter_type[shelter_id] = 1;
                                shelters[shelter_id]
                                    = get_shelter_aginst_point(shelter_id, robots[fix_id].loc);
                                left_shelter_cnt[shelter_id] = 1;
                            }
                        }
                    }
                    else
                    {

                        double delta_angle = fabs(robots[i].dirc - robots[j].dirc);
                        if (delta_angle > M_PI)
                            delta_angle -= 2 * M_PI;
                        else if (delta_angle < -M_PI)
                            delta_angle += 2 * M_PI;
                        if (fabs(delta_angle) < M_PI / 9)
                        {
                            int stop_index = robots[i].v.len() < robots[j].v.len() ? i : j;
                            shelter_type[stop_index] = 2;
                            left_shelter_cnt[stop_index] = persisitent_flame;
                        }
                        else
                        {
                            shelter_type[i] = 1;
                            shelter_type[j] = 1;
                            shelters[i] = get_shelter_aginst_point(i, robots[j].loc);
                            shelters[j] = get_shelter_aginst_point(j, robots[i].loc);
                            left_shelter_cnt[i] = 1;
                            left_shelter_cnt[j] = 1;
                        }
                    }
                }
            }
        }
    }

    for (int i = 1; i < shelters.size(); ++i)
    {
        if (left_shelter_cnt[i] == 0)
        {
            continue;
        }
        cerr << "info: robot " << i << " v " << robots[i].v << " w " << robots[i].w << " pos "
             << robots[i].loc << endl;
        if (shelter_type[i] == 2)
        {

            cerr << "info robot " << i << " will stop" << endl;
            instructions.push_back(new io::I_forward(i, 0));
            continue;
        }
        if (shelter_type[i] == 1)
        {
            cerr << "info: robot " << i << " will be assigned shelter: " << shelters[i] << " left "
                 << left_shelter_cnt[i] << " flame(s)." << endl;

            navigate::move_to(robots[i], shelters[i]);
            continue;
        }
    }
}
}

#endif