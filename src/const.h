#ifndef __CONST_H__
#define __CONST_H__
#define _USE_MATH_DEFINES
#include <cmath>

/*题目常量 constant variable*/
namespace ConVar
{
const static int time_limit = 3 * 60 * 50;             // 3分钟
const static double map_weight = 50.0;                 // 地图大小
const static double map_height = 50.0;                 // 地图大小
const static int worksation_type = 9;                  // 工作台种类
const static int max_workstation = 50;                 // 工作台数量
const static int good_types = 7;                       // 货物种类
const static int max_robot = 4;                        // 机器人数量
const static double robot_radius = 0.5;                // 机器人半径
const static double robot_radius_goods = 0.53;         // 取货机器人半径
const static double robot_workstation_check = 0.4;     // 工作台检测半径
const static double robot_density = 20;                // 机器人密度
const static double max_robot_forward_speed = 6;       // 机器人最大前进速度
const static double max_robot_backward_speed = 2;      // 机器人最大后退速度
const static double max_robot_angular_speed = M_PI;    // 机器人最大角速度
const static double max_force = 250;                   // 最大牵引力
const static double max_torque = 60;                   // 最大扭矩
};
/*常用值 comman varaible*/
namespace ComVar
{
const static double robot_weight
    = ConVar::robot_radius * ConVar::robot_radius * M_PI * ConVar::robot_density;    // 机器人质量
const static double robot_goods_weight = ConVar::robot_radius_goods * ConVar::robot_radius_goods * M_PI
    * ConVar::robot_density;                                                      // 机器人质量
const static double max_robot_acceleration = ConVar::max_force / robot_weight;    // 最大加速度
const static double max_robot_goods_acceleration = ConVar::max_force / robot_goods_weight;    // 最大加速度
const static double flametime = 1.0 / 50;    // 一帧时间
}

#endif