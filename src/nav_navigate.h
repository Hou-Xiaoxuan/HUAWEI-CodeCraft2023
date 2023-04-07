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

double __normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
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

    // 已经冲过了 停下来
    if (Vertex::distance(stop_loc, path[0]) > 1e-2 and abs(angle) < M_PI / 9)
    {
        cerr << "robot " << robot.id << " stop over" << endl;
        instructions.push_back(new io::I_forward(robot.id, 0));
        return;
    }

    // 距离目标很近停下来
    if (Vertex::distance(stop_loc, path[1]) < 2e-1)
    {
        cerr << "[info][__change_speed] robot " << robot.id << " stop 接近目标" << endl;
        instructions.push_back(new io::I_forward(robot.id, 0));
        return;
    }

    double delta = __get_delta_angle(robot, path[1]);

    if (abs(delta) > M_PI / 18 and robot.w > M_PI / 18)
    {
        cerr << "robot " << robot.id << " stop rotate" << endl;
        instructions.push_back(new io::I_forward(robot.id, 0));
        return;
    }

    instructions.push_back(new io::I_forward(robot.id, ConVar::max_robot_forward_speed));
}

/*角度调整*/
void __change_direction(const Robot &robot, const vector<Vertex> &path)
{
    Vertex target = path[1];

    double y = target.y - robot.loc.y;
    double x = target.x - robot.loc.x;

    double right_angle = atan2(y, x);

    double delta = __get_delta_angle(robot, target);

    double angular_acceleration_symbol
        = __get_max_robot_angular_acceleration(robot) * (signbit(robot.w) == 1 ? -1 : 1);
    double angular_acceleration_rev_symbol = -angular_acceleration_symbol;

    double stop_t = -robot.w / angular_acceleration_rev_symbol;

    if (stop_t < 0)
    {
        cerr << "robot " << robot.id << " throw error t < 0" << endl;
        throw "error t < 0";
    }

    double stop_angular_1
        = robot.dirc + robot.w * stop_t + 0.5 * angular_acceleration_rev_symbol * stop_t * stop_t;
    stop_angular_1 = __normalize_angle(stop_angular_1);

    // 立刻停止，判断是否来不及了0
    if (abs(__normalize_angle(robot.dirc - right_angle))
            < abs(__normalize_angle(robot.dirc - stop_angular_1))
        and abs(__normalize_angle(right_angle - stop_angular_1))
            < abs(__normalize_angle(robot.dirc - stop_angular_1)))
    {
        cerr << "robot " << robot.id << " rotate to logic 0" << endl;
        instructions.push_back(new io::I_rotate(robot.id, 0));
        return;
    }

    // 立即停止，判断是否正好1
    if (abs(__normalize_angle(stop_angular_1 - right_angle)) < M_PI / 90)
    {
        cerr << "robot " << robot.id << " rotate to logic 1" << endl;
        instructions.push_back(new io::I_rotate(robot.id, 0));
        return;
    }

    // 判断再走一帧是否正好2
    double small_delta = robot.w * ComVar::flametime;
    double stop_angular_2 = stop_angular_1 + small_delta;
    stop_angular_2 = __normalize_angle(stop_angular_2);
    if (abs(__normalize_angle(stop_angular_2 - right_angle)) < M_PI / 90)
    {
        cerr << "robot " << robot.id << " rotate to logic 2" << endl;
        instructions.push_back(new io::I_rotate(robot.id, robot.w));
        return;
    }


    // 是否在上面的两个区间内3
    // 如果是先减速 再平动 在减速
    if (abs(__normalize_angle(stop_angular_1 - right_angle)) < abs(small_delta)
        and abs(__normalize_angle(stop_angular_2 - right_angle)) < abs(small_delta))
    {
        double nw_1 = 0.5 * (angular_acceleration_rev_symbol * ComVar::flametime + robot.w);
        double nw_2 = 0.5
            * sqrt(angular_acceleration_rev_symbol * angular_acceleration_rev_symbol * ComVar::flametime
                    * ComVar::flametime
                - 4 * angular_acceleration_rev_symbol * delta
                + 2 * angular_acceleration_rev_symbol * ComVar::flametime * robot.w - robot.w * robot.w);

        if (nw_1 + nw_2 > robot.w
                and nw_1 + nw_2 < robot.w + angular_acceleration_rev_symbol * ComVar::flametime
            or nw_1 + nw_2 < robot.w
                and nw_1 + nw_1 > robot.w + angular_acceleration_rev_symbol * ComVar::flametime)
        {
            instructions.push_back(new io::I_rotate(robot.id, nw_1 + nw_2));
            cerr << "robot " << robot.id << " rotate to logic 3 +" << endl;
            return;
        }
        else if (nw_1 - nw_2 > robot.w
                and nw_1 - nw_2 < robot.w + angular_acceleration_rev_symbol * ComVar::flametime
            or nw_1 - nw_2 < robot.w
                and nw_1 - nw_1 > robot.w + angular_acceleration_rev_symbol * ComVar::flametime)
        {
            instructions.push_back(new io::I_rotate(robot.id, nw_1 - nw_2));
            cerr << "robot " << robot.id << " rotate to logic 3 -" << endl;
            return;
        }
        else
        {
            cerr << "robot " << robot.id << " throw logic 3 error" << endl;
            throw "logic 3 error";
            return;
        }
    }

    // 加速到极限再立刻减速
    // if (abs(robot.w) < 1e-2) {

    // }
    double next_w_1 = sqrt(angular_acceleration_symbol * delta + robot.w * robot.w * 0.5)
        * (signbit(delta) == 1 ? -1 : 1);

    // if (abs(next_w_1) < abs(robot.w))
    // {
    //     cerr << "robot " << robot.id << " throw logic 4 error" << endl;
    //     throw "logic 4 error";
    // }

    if ((next_w_1 - robot.w) / angular_acceleration_symbol >= ComVar::flametime
        or abs(next_w_1) >= ConVar::max_robot_angular_speed)
    {
        instructions.push_back(new io::I_rotate(robot.id, next_w_1));
        cerr << "robot " << robot.id << " rotate to logic 4" << endl;
        return;
    }

    double next_w_2 = (angular_acceleration_symbol * delta + 0.5 * robot.w * robot.w)
        / (robot.w + angular_acceleration_symbol * ComVar::flametime);

    instructions.push_back(new io::I_rotate(robot.id, next_w_2));
    cerr << "robot " << robot.id << " rotate to logic 5" << endl;

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