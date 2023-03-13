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
/*速度调整*/
void __change_speed(const Robot& robot, const Point &target, const vector<Point> &follow_target){}

/*角度调整*/
void __change_direction(const Robot &robot, const Point &target, const vector<Point> &follow_target) { }


/*将i号机器人移动到target点
 * 可选参数follow_target, 考虑后续目标点的航迹优化
 */
void move_to(int robot_id, Point target, vector<Point> follow_target = vector<Point>()) { 
    const auto &robot = meta.robot[robot_id];
    // 已经到达目标点
    if(Point::distance(robot.loc, target) < ConVar::robot_workstation_check*0.7) {
        instructions.push_back(new io::I_forward(robot_id, 0));
        __change_direction(robot, target, follow_target);
    }

    
}
}
#endif