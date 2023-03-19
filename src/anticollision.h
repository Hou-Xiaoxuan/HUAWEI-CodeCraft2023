#ifndef __ANTICOLLISION_H__
#define __ANTICOLLISION_H__

#include <optional>
#include <vector>
using namespace std;
#include "const.h"
#include "model.h"
#include "route_fool.h"

namespace anticollision
{
using io::instructions;

void __shelter(const Robot &robot, int RotateDirection)
{
    Point center;    // 圆心
    center.x = robot.loc.x + RotateDirection * ComVar::max_radius * sin(robot.dirc);
    center.y = robot.loc.y - RotateDirection * ComVar::max_radius * cos(robot.dirc);

    double shelter_angular = robot.dirc + M_PI / 2;
    Point shelter_point;
    shelter_point.x = center.x - RotateDirection * ComVar::max_radius * sin(shelter_angular);
    shelter_point.y = center.y + RotateDirection * ComVar::max_radius * cos(shelter_angular);

    navigate::move_to(robot, shelter_point);
}
// vector<vector<Point>> all_predict_point(ConVar::max_robot + 1, vector<Point>(ConVar::time_limit + 1));
void anticollision(const vector<optional<route_fool::Route>> &route)
{

    for (int predict_flame = 1; predict_flame <= 20; predict_flame += 1)
    {
        vector<Point> predict_point(meta.robot.size());
        double predict_time = predict_flame * ComVar::flametime;
        for (int i = 1; i < meta.robot.size(); ++i)
        {
            // cerr << "info: "
            //      << "robot " << i << " speed: " << meta.robot[i].v << endl;
            const auto &robot = meta.robot[i];

            if (fabs(robot.w) < M_PI / 10)
            {
                predict_point[i].x = robot.loc.x + robot.v.x * predict_time;
                predict_point[i].y = robot.loc.y + robot.v.y * predict_time;

                // all_predict_point[i][meta.current_flame] = predict_point[i];
                //  if (meta.current_flame > predict_flame && i == 1)
                //  {
                //      double error
                //          = Point::distance(all_predict_point[i][meta.current_flame - predict_flame],
                //          robot.loc);
                //      // if (error > 0.4)
                //      // {
                //      cerr << "info: "
                //           << "robot " << i << " line predict error: " << error << endl;
                //      // }
                //  }
                continue;
            }

            double radius = robot.v.len() / robot.w;    // 半径
            Point center;                               // 圆心
            center.x = robot.loc.x - radius * sin(robot.dirc);
            center.y = robot.loc.y + radius * cos(robot.dirc);

            double predict_angular = robot.dirc + robot.w * predict_time;
            predict_point[i].x = center.x + radius * sin(predict_angular);
            predict_point[i].y = center.y - radius * cos(predict_angular);
            // all_predict_point[i][meta.current_flame] = predict_point[i];
            //  cerr << "info: "
            //       << "robot " << i << " center point: " << center << endl;
            //  cerr << "info: "
            //       << "robot " << i << " center radius: " << radius << endl;

            // if (meta.current_flame > predict_flame && i == 1)
            // {
            //     double error
            //         = Point::distance(all_predict_point[i][meta.current_flame - predict_flame],
            //         robot.loc);
            //     // if (error > 0.4)
            //     // {
            //     cerr << "info: "
            //          << "robot " << i << " circle predict error: " << error << endl;

            //     // }
            // }
        }

        for (int i = 1; i < predict_point.size(); ++i)
        {
            for (int j = i + 1; j < predict_point.size(); ++j)
            {
                double min_distance = navigate::__get_robot_radius(meta.robot[i])
                    + navigate::__get_robot_radius(meta.robot[j]);
                if (Point::distance(predict_point[i], predict_point[j]) <= min_distance)
                {
                    cerr << "info: "
                         << "robot " << i << " and robot " << j << " will collide" << endl;
                    double delta = meta.robot[i].dirc - meta.robot[j].dirc;
                    if (delta > M_PI)
                        delta -= 2 * M_PI;
                    else if (delta < -M_PI)
                        delta += 2 * M_PI;
                    // XXX Check
                    if (fabs(delta) < M_PI / 2)
                    {
                        int flag = signbit(delta) ? 1 : -1;
                        __shelter(meta.robot[i], flag);
                        __shelter(meta.robot[j], -flag);
                    }
                    else
                    {
                        __shelter(meta.robot[i], 1);
                        __shelter(meta.robot[j], 1);
                    }
                }
            }
        }
    }
}
}

#endif