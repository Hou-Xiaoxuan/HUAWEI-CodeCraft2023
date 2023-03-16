#ifndef __NAVIGATE_H__
#define __NAVIGATE_H__
#include "const.h"
#include "iointerface.h"
#include "model.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
/* 控制机器人导航*/
namespace navigate
{
using namespace std;
using io::instructions;
double __get_max_robot_acceleration(const Robot &robot) { return ComVar::max_robot_goods_acceleration; }


double __get_delta_angle(const Robot &robot, const Point &target)
{
    double y = target.y - robot.loc.y;
    double x = target.x - robot.loc.x;

    double angle = atan2(y, x);

    double delta = angle - robot.dirc;
    if (delta > M_PI)
        delta -= 2 * M_PI;
    else if (delta < -M_PI)
        delta += 2 * M_PI;
    return delta;
}

double __get_robot_radius(const Robot &robot)
{
    return robot.goods == 0 ? ConVar::robot_radius : ConVar::robot_radius_goods;
}

bool __is_in_circle(const Robot &robot, const Point &target)
{

    // 顺时针 x + r * sin y - r * cos
    // 逆时针 x - r * sin y + r * cos
    Point center;
    // < 0 顺时针; > 0 逆时针
    int flag = signbit(__get_delta_angle(robot, target)) ? 1 : -1;
    center.x = robot.loc.x + flag * ComVar::max_ridus * sin(robot.dirc);
    center.y = robot.loc.y - flag * ComVar::max_ridus * cos(robot.dirc);

    double radius = __get_robot_radius(robot);
    return Point::distance(center, target) <= ComVar::max_ridus - radius;
}


/*速度调整*/
void __change_speed(const Robot &robot, const Point &target, const vector<Point> &follow_target)
{

    // prevent from wall
    double min_a_x = __get_max_robot_acceleration(robot) * robot.v.x / robot.v.len();
    double min_a_y = __get_max_robot_acceleration(robot) * robot.v.y / robot.v.len();
    double t = robot.v.len() / __get_max_robot_acceleration(robot);
    double stop_x = robot.loc.x + robot.v.x * t + 0.5 * min_a_x * t * t;
    double stop_y = robot.loc.y + robot.v.y * t + 0.5 * min_a_y * t * t;

    if (stop_x - ConVar::robot_radius_goods <= 0
        || stop_x + ConVar::robot_radius_goods >= ConVar::map_weight
        || stop_y - ConVar::robot_radius_goods <= 0
        || stop_y + ConVar::robot_radius_goods >= ConVar::map_height)
    {
        instructions.push_back(new io::I_forward(robot.id, 0));
        return;
    }

    // prevent from circle
    if (__is_in_circle(robot, target))
    {
        double next_v = 0.5 * Point::distance(robot.loc, target) / sin(__get_delta_angle(robot, target))
            * ConVar::max_robot_angular_speed;
        instructions.push_back(new io::I_forward(robot.id, next_v));
        return;
    }
    instructions.push_back(new io::I_forward(robot.id, ConVar::max_robot_forward_speed));
}

/*角度调整*/
void __change_direction(const Robot &robot, const Point &target, const vector<Point> &follow_target)
{
    double delta = __get_delta_angle(robot, target);
    if (delta >= ComVar::flametime * ConVar::max_robot_angular_speed)
    {
        instructions.push_back(
            new io::I_rotate(robot.id, ConVar::max_robot_angular_speed * (signbit(delta) ? -1 : 1)));
    }
    else
    {
        instructions.push_back(new io::I_rotate(robot.id, delta / ComVar::flametime));
    }
}


/*将i号机器人移动到target点
 * 可选参数follow_target, 考虑后续目标点的航迹优化
 */
void move_to(const Robot &robot, Point target, vector<Point> follow_target = vector<Point>())
{
    __change_direction(robot, target, follow_target);
    __change_speed(robot, target, follow_target);
    io::print_instructions(instructions, cout, meta.current_flame);
}
}
#endif