#ifndef __NAV_NAVIGATE_H__
#define __NAV_NAVIGATE_H__
#include "const.h"
#include "find_path_squre.h"
#include "iointerface.h"
#include "model.h"
#include "nav_model.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
/* 控制机器人导航*/
namespace nav_navigate
{
using namespace navmesh;
using namespace std;
using io::instructions;

double __get_max_robot_acceleration(const Robot &robot)
{
    return robot.goods == 0 ? ComVar::max_robot_acceleration : ComVar::max_robot_goods_acceleration;
}

double __get_robot_radius(const Robot &robot)
{
    return robot.goods == 0 ? ConVar::robot_radius : ConVar::robot_radius_goods;
}

double __get_max_robot_angular_acceleration(const Robot &robot)
{
    return robot.goods == 0 ? ComVar::max_robot_angular_acceleration
                            : ComVar::max_robot_angular_acceleration_with_goods;
}

double __get_delta_angle(const Robot &robot, const Vertex &target)
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

// Vertex __stop_with_spec_w(const Robot &robot)
// {
//     double w = robot.w;
//     double a = __get_max_robot_angular_acceleration(robot);
//     double v_0 = robot.v.len();
//     double t = v_0 / a;
//     double p_0 = robot.dirc;

//     double x = robot.loc.x - (a * (cos(t * w + p_0) - cos(p_0)) + robot.v.y * w) / (w * w);
//     double y = robot.loc.y + (a * (sin(p_0) - sin(t * w + p_0)) + robot.v.x * w) / (w * w);
//     return {x, y};
// }

// bool __is_in_circle(const Robot &robot, const Vertex &target)
// {
//     // 顺时针 x - r * sin y + r * cos
//     // 逆时针 x + r * sin y - r * cos
//     Vertex center;
//     // < 0 顺时针; > 0 逆时针
//     int flag = signbit(__get_delta_angle(robot, target)) ? -1 : 1;
//     center.x = robot.loc.x - flag * ComVar::max_radius * sin(robot.dirc);
//     center.y = robot.loc.y + flag * ComVar::max_radius * cos(robot.dirc);

//     double radius = __get_robot_radius(robot);
//     return Vertex::distance(center, target) + ConVar::robot_workstation_check <= ComVar::max_radius;
// }

/*速度调整*/
void __change_speed(const Robot &robot, const vector<Vertex> &path)
{
    // double rorate_limit = M_PI / 18;

    // prevent from circle
    // if (__is_in_circle(robot, target))
    // {
    //     instructions.push_back(new io::I_forward(robot.id, 0));
    //     return;
    // }

    double min_a_x = __get_max_robot_acceleration(robot) * robot.v.x / robot.v.len();
    double min_a_y = __get_max_robot_acceleration(robot) * robot.v.y / robot.v.len();
    double stop_t = robot.v.len() / __get_max_robot_acceleration(robot);
    double stop_x = robot.loc.x + robot.v.x * stop_t + 0.5 * min_a_x * stop_t * stop_t;
    double stop_y = robot.loc.y + robot.v.y * stop_t + 0.5 * min_a_y * stop_t * stop_t;
    Vertex stop_loc = {stop_x, stop_y};

    Vec2 vec1 = {path[0], stop_loc};
    Vec2 vec2 = {path[1], stop_loc};
    double angle = Vec2::angle(vec1, vec2);
    if (Vertex::distance(stop_loc, path[1]) < 0.05)
    {
        cerr << "[info][__change_speed] robot " << robot.id << " stop 接近目标" << endl;
        instructions.push_back(new io::I_forward(robot.id, 0));
        return;
    }
    if (Vertex::distance(stop_loc, path[0]) > 1e-7 and abs(angle) < M_PI / 9)
    {
        cerr << "[info][__change_speed] robot " << robot.id << " stop 越过目标" << endl;
        instructions.push_back(new io::I_forward(robot.id, 0));
        return;
    }

    double delta = __get_delta_angle(robot, path[1]);

    if (abs(delta) > M_PI / 10)
    {
        cerr << "[info][__change_speed] robot " << robot.id << " stop 方向不对" << endl;
        instructions.push_back(new io::I_forward(robot.id, 0));
        return;
    }

    instructions.push_back(new io::I_forward(robot.id, ConVar::max_robot_forward_speed));
}

/*角度调整*/
void __change_direction(const Robot &robot, const vector<Vertex> &path)
{
    Vertex target = path[1];
    double angular_acceleration = __get_max_robot_angular_acceleration(robot);
    Vertex next_target = target;

    // if (path.size() >= 3 && Vertex::distance(robot.loc, target) < 0.1 && robot.v.len() < 1e-2)
    // {
    //     next_target = path[2];
    // }

    double delta = __get_delta_angle(robot, next_target);
    // cerr << "info: robot " << robot.id << " dir " << robot.dirc << " delta " << delta << endl;

    int delta_dir = signbit(delta) ? -1 : 1;


    if (signbit(delta) != signbit(robot.w))    // HACK
    {
        instructions.push_back(new io::I_rotate(robot.id, ConVar::max_robot_angular_speed * delta_dir));
        return;
    }

    delta = abs(delta);

    double stop_angular = robot.w * robot.w * 0.5 / angular_acceleration;
    if (stop_angular >= delta)
    {
        instructions.push_back(new io::I_rotate(robot.id, 0));
        return;
    }


    stop_angular = abs(robot.w) * ComVar::flametime + stop_angular;
    if (stop_angular >= delta)
    {
        instructions.push_back(new io::I_rotate(robot.id, robot.w));
        return;
    }

    double wm = (robot.w
                    + (signbit(robot.w) == 1 ? -1 : 1)
                        * sqrt(4 * angular_acceleration * delta - robot.w * robot.w))
        * 0.5;

    if (abs(wm) > ConVar::max_robot_angular_speed)
    {
        instructions.push_back(new io::I_rotate(robot.id, ConVar::max_robot_angular_speed * delta_dir));
        return;
    }

    instructions.push_back(new io::I_rotate(robot.id, wm));
    return;
}


/*将i号机器人移动到target点
 * 可选参数follow_target, 考虑后续目标点的航迹优化
 */
void move_to(const Robot &robot, vector<Vertex> path)
{
    if (not path.empty())
    {
        cerr << "[info][move_to]: robot " << robot.id << " move to: ";
        for (auto &p : path)
            cerr << p << "->";
        cerr << endl;
        __change_direction(robot, path);
        __change_speed(robot, path);
    }
    else
    {
        cerr << "[info][move_to]: robot " << robot.id << " move to: empty" << endl;
        instructions.push_back(new io::I_forward(robot.id, 0));
        instructions.push_back(new io::I_rotate(robot.id, 0));
    }
}
}
#endif