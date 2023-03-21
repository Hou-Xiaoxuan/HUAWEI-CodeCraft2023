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

Point get_spec_dir_shelter(int robot_id, int RotateDirection)
{
    const auto &robot = meta.robot[robot_id];
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


void anticollision(const vector<optional<route_fool::Route>> &routes)
{

    int max_predict_flame = 15;
    const auto &robots = meta.robot;
    vector<optional<Point>> shelters(robots.size());
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

                double min_distance
                    = navigate::__get_robot_radius(robots[i]) + navigate::__get_robot_radius(robots[j]);
                if (Point::distance(predict_point[predict_flame][i], predict_point[predict_flame][j])
                    <= min_distance)
                {
                    cerr << "info: robot " << i << " and robot " << j << " will collide in "
                         << predict_flame << " flame(s)." << endl;

                    vis[i][j] = 1;

                    if (shelters[i].has_value() && shelters[j].has_value())
                    {
                        cerr << "info: robot " << i << " and robot " << j
                             << " have already been assigned shelters." << endl;
                        cerr << "info: robot fate is decided." << endl;
                        continue;
                    }

                    else if (shelters[i].has_value() || shelters[j].has_value())
                    {
                        const Point fix_shelter
                            = shelters[i].has_value() ? shelters[i].value() : shelters[j].value();
                        int shelter_id = shelters[i].has_value() ? j : i;
                        shelters[shelter_id] = get_shelter_aginst_point(shelter_id, fix_shelter);
                    }
                    else
                    {
                        vector<Point> shelter_i = {
                            get_spec_dir_shelter(robots[i].id, 1), get_spec_dir_shelter(robots[i].id, -1)};
                        vector<Point> shelter_j = {
                            get_spec_dir_shelter(robots[j].id, 1), get_spec_dir_shelter(robots[j].id, -1)};
                        int shelter_index_i = 0, shelter_index_j = 0;
                        double shelter_dis = 0;
                        for (int k = 0; k < 2; ++k)
                        {
                            for (int l = 0; l < 2; ++l)
                            {
                                double tmp_dis = Point::distance(shelter_i[k], shelter_j[l]);
                                cerr << "info: robot " << i << " shelter " << shelter_i[k] << " and robot "
                                     << j << " shelter " << shelter_j[l] << " distance is " << tmp_dis
                                     << endl;
                                if (tmp_dis >= shelter_dis)
                                {
                                    shelter_dis = tmp_dis;
                                    shelter_index_i = k;
                                    shelter_index_j = l;
                                }
                            }
                        }
                        shelters[i] = shelter_i[shelter_index_i];
                        shelters[j] = shelter_j[shelter_index_j];
                    }
                }
            }
        }
    }

    for (int i = 1; i < shelters.size(); ++i)
    {
        if (shelters[i].has_value())
        {
            cerr << "info: robot " << i << " will be assigned shelter: " << shelters[i].value() << endl;
            cerr << "info: robot " << i << " v " << robots[i].v << " w " << robots[i].w << " pos "
                 << robots[i].loc << endl;
            navigate::move_to(robots[i], shelters[i].value());
        }
    }
}
}

#endif