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

vector<io::Instruction *> instructions;

double __get_max_robot_acceleration(const Robot &robot) { return ComVar::max_robot_goods_acceleration; }


/*速度调整*/
void __change_speed(const Robot &robot, const Point &target, const vector<Point> &follow_target)
{
    double next_v;
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
        next_v = 0;
    }
    else
    {
        next_v = ConVar::max_robot_forward_speed;
    }
    instructions.push_back(new io::I_forward(robot.id, next_v));
}

/*角度调整*/
void __change_direction(const Robot &robot, const Point &target, const vector<Point> &follow_target)
{
    double y = target.y - robot.loc.y;
    double x = target.x - robot.loc.x;

    double angle = atan2(y, x);

    double delta = angle - robot.dirc;
    if (delta > M_PI)
        delta -= 2 * M_PI;
    else if (delta < -M_PI)
        delta += 2 * M_PI;
    if (delta >= ComVar::flametime * ConVar::max_robot_angular_speed)
    {
        instructions.push_back(new io::I_rotate(robot.id, ConVar::max_robot_angular_speed));
    }
    else
    {
        instructions.push_back(new io::I_rotate(robot.id, delta / ComVar::flametime));
    }
}


/*将i号机器人移动到target点
 * 可选参数follow_target, 考虑后续目标点的航迹优化
 */
void move_to(int robot_id, Point target, vector<Point> follow_target = vector<Point>())
{
    const auto &robot = meta.robot[robot_id];
    // 已经到达目标点
    if (Point::distance(robot.loc, target) < ConVar::robot_workstation_check * 0.7)
    {
        instructions.push_back(new io::I_forward(robot_id, 0));
    }
    __change_direction(robot, target, follow_target);
    __change_speed(robot, target, follow_target);
}
}
#endif