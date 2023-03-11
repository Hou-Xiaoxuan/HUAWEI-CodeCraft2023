#ifndef __CONST_H__
#define __CONST_H__
// const int TIME_LIMIT = 3 * 60 * 50;    // 3分钟
// const double MAP_WEIGHT = 50.0;        // 地图大小
// const double MAP_HEIGHT = 50.0;        // 地图大小
// const int MAX_WORKSTATION = 9;         // 工作台数量
// const int MAX_GOODS = 7;               // 货物数量

// const int MAX_ROBOT = 4;                       // 机器人数量
// const double ROBOT_RADIUS = 0.5;               // 机器人半径
// const double ROBOT_RADIUS_GOODS = 0.53;        // 取货机器人半径
// const double ROBOT_WORKSTATION_CHECK = 0.4;    // 工作台检测半径
// const double ROBOT_DENSITY = 20;               // 机器人密度
struct ConVar
{
    static const int time_limit = 3 * 60 * 50;    // 3分钟
    static const double map_weight = 50.0;        // 地图大小
    static const double map_height = 50.0;        // 地图大小
    static const int max_workstation = 9;         // 工作台数量
    static const int max_goods = 7;               // 货物数量
    
    static const int max_robot = 4;                       // 机器人数量
    static const double robot_radius = 0.5;               // 机器人半径
    static const double robot_radius_goods = 0.53;        // 取货机器人半径
    static const double robot_workstation_check = 0.4;    // 工作台检测半径
    static const double robot_density = 20;               // 机器人密度
};

#endif