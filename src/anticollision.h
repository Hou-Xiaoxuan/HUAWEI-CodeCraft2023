#ifndef __ANTICOLLISION_H__
#define __ANTICOLLISION_H__

#include <optional>
#include <iostream>
#include <vector>
#include "const.h"
#include "model.h"
#include "route_fool.h"
#include <set>

namespace anticollision
{
using namespace std;
using io::instructions;

void _shelter(const Robot &robot, int RotateDirection)
{
    Point center;    // 圆心
    center.x = robot.loc.x + RotateDirection * ComVar::max_radius * sin(robot.dirc);
    center.y = robot.loc.y - RotateDirection * ComVar::max_radius * cos(robot.dirc);

    double shelter_angular = robot.dirc + M_PI / 2;
    Point shelter_point;
    shelter_point.x = center.x - RotateDirection * ComVar::max_radius * sin(shelter_angular);
    shelter_point.y = center.y + RotateDirection * ComVar::max_radius * cos(shelter_angular);
    cerr << "info: "
         << "robot " << robot.id << " shelter point: " << shelter_point << endl;

    navigate::move_to(robot, shelter_point);
}

void anticollision(const vector<optional<route_fool::Route>> &routes)
{

    set<pair<int, int>> collision_set;
    const auto &robots = meta.robot;
    for (int predict_flame = 1; predict_flame <= 20; predict_flame += 1)
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
        double delta = robots[i].dirc - robots[j].dirc;
        if (delta > M_PI)
            delta -= 2 * M_PI;
        else if (delta < -M_PI)
            delta += 2 * M_PI;

        cerr << "info: "
             << "robot " << i << " and robot " << j << " delta " << delta << endl;
        if (fabs(delta) < M_PI / 2)
        {
            cerr << "info: "
                 << "robot " << i << " and robot " << j << " rotate different" << endl;
            int flag = signbit(delta) ? 1 : -1;
            _shelter(meta.robot[i], flag);
            _shelter(meta.robot[j], -flag);
        }
        else
        {
            cerr << "info: "
                 << "robot " << i << " and robot " << j << " rotate same" << endl;
            _shelter(meta.robot[i], 1);
            _shelter(meta.robot[j], 1);
        }
    }
}
}

#endif