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
Point _shelter(const Robot &robot, int RotateDirection)
{
    Point center;    // 圆心
    center.x = robot.loc.x + RotateDirection * ComVar::max_radius * sin(robot.dirc);
    center.y = robot.loc.y - RotateDirection * ComVar::max_radius * cos(robot.dirc);

    double shelter_angular = robot.dirc + M_PI / 2;
    Point shelter_point;
    shelter_point.x = center.x - RotateDirection * ComVar::max_radius * sin(shelter_angular);
    shelter_point.y = center.y + RotateDirection * ComVar::max_radius * cos(shelter_angular);
    return shelter_point;
}

void anticollision(const vector<optional<route_fool::Route>> &routes)
{

    set
    <pair<int, int>> collision_set;
    const auto &robots = meta.robot;
    for (int predict_flame = 15; predict_flame > 0; predict_flame -= 1)
    {
        vector<Point> predict_point(robots.size());
        double predict_time = predict_flame * ComVar::flametime;
        for (int i = 1; i < robots.size(); ++i)
        {
            const auto &robot = robots[i];

            if (fabs(robot.w) < M_PI / 10)
            {
                predict_point[i].x = robot.loc.x + robot.v.x * predict_time;
                predict_point[i].y = robot.loc.y + robot.v.y * predict_time;
                continue;
            }

            double radius = robot.v.len() / robot.w;    // 半径
            Point center;                               // 圆心
            center.x = robot.loc.x - radius * sin(robot.dirc);
            center.y = robot.loc.y + radius * cos(robot.dirc);

            double predict_angular = robot.dirc + robot.w * predict_time;
            predict_point[i].x = center.x + radius * sin(predict_angular);
            predict_point[i].y = center.y - radius * cos(predict_angular);
        }

        for (int i = 1; i < predict_point.size(); ++i)
        {
            for (int j = i + 1; j < predict_point.size(); ++j)
            {

                double min_distance
                    = navigate::__get_robot_radius(robots[i]) + navigate::__get_robot_radius(robots[j]);
                if (Point::distance(predict_point[i], predict_point[j]) <= min_distance)
                {
                    collision_set.insert({i, j});
                }
            }
        }
    }

    // bianli collision_set
    for (const auto &collision : collision_set)
    {
        int i = collision.first;
        int j = collision.second;

        cerr << "info: "
             << "robot " << i << " and robot " << j << " will collide" << endl;
        vector<Point> shelter_i = {_shelter(robots[i], 1), _shelter(robots[i], -1)};
        vector<Point> shelter_j = {_shelter(robots[j], 1), _shelter(robots[j], -1)};
        int shelter_index_i = 0, shelter_index_j = 0;
        double shelter_dis = 0;
        for (int k = 0; k < 2; ++k)
        {
            for (int l = 0; l < 2; ++l)
            {
                double tmp_dis = Point::distance(shelter_i[k], shelter_j[l]);
                if (tmp_dis >= shelter_dis)
                {
                    shelter_dis = tmp_dis;
                    shelter_index_i = k;
                    shelter_index_j = l;
                    break;
                }
            }
        }
        navigate::move_to(robots[i], shelter_i[shelter_index_i]);
        navigate::move_to(robots[j], shelter_j[shelter_index_j]);
        cerr << "info: "
             << "robot " << i << " move to " << shelter_i[shelter_index_i] << endl;
        cerr << "info: "
             << "robot " << j << " move to " << shelter_j[shelter_index_j] << endl;
    }
}
}

#endif