#ifndef _TEST_TRIANGLE_H_
#define _TEST_TRIANGLE_H_
#include "const.h"
#include "iointerface.h"
#include "model.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
/** 测试控制机器人做三角运动 */
namespace test_triangle
{
using namespace std;

vector<int> robot_target(5, -1);    // 机器人目标点

int next_target(int st)
{
    // 10 20 30 号点之间循环
    if (st == 1)
        return 2;
    else if (st == 2)
        return 3;
    else if (st == 3)
        return 1;
    else
        return 1;
}
/*处理本帧的输出*/
void process_flame()
{
    vector<io::Instruction *> instructions;
    int robot_id = 1;    // 使用1号机器人实验
    const auto &robot = meta.robot[robot_id];
    if (robot_target[robot_id] == -1)
        robot_target[robot_id] = next_target(-1);    // 初始化目标点
    auto target = meta.station[robot_target[robot_id]].loc;
    // robot转向目标点的夹角，x轴正方向为0度，顺时针为正
    double angle = atan2(target.y - robot.loc.y, target.x - robot.loc.x);
    if (fabs(angle - robot.dirc) > 1e-6)
    {
        // // 速度置为0, 旋转到目标点
        // instructions.push_back(new io::I_forward(robot_id, 0));
        double angle_diff = angle - robot.dirc;
        if (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
        else if (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;
        if (angle_diff < ComVar::flametime * ConVar::max_robot_angular_speed)
            instructions.push_back(new io::I_rotate(robot_id, angle_diff / ComVar::flametime));
        else
            instructions.push_back(new io::I_rotate(robot_id, ConVar::max_robot_angular_speed));
    }
    if(true)
    {
        // 机器人前进
        instructions.push_back(new io::I_forward(robot_id, 6));
        // instructions.push_back(new io::I_rotate(robot_id, 0));
        // 到达目标点
        // auto dis = Point::distance(robot.loc, target);
        
        if (robot.in_station == robot_target[robot_id]-1)
        {
            cerr << "info: robot " << robot_id << " arrive target " << robot_target[robot_id] << endl;
            robot_target[robot_id] = next_target(robot_target[robot_id]);
            instructions.push_back(new io::I_forward(robot_id, 0));
        }
        else
        {
            // 根据速度和加速度规划运动，尽量减少停止时间
            // 1. 计算当前速度
            double speed = sqrt(robot.v.x * robot.v.x + robot.v.y * robot.v.y);
            // 2. 计算当前加速度
            double dis = Point::distance(robot.loc, target);
            double need_time = dis / speed;
            if (dis > 0.5 * speed * speed / ComVar::max_robot_acceleration)
            { }
            else
            {
                instructions.push_back(new io::I_forward(robot_id, speed*0.7));
            }
        }
    }

    io::print_instructions(instructions, cout, meta.current_flame);
    return;
}

}
#endif