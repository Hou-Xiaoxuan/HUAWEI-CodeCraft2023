#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"
#include "navigate.h"

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

int main()
{
    init(cin);
    puts("OK");
    fflush(stdout);

    /*----------START----------*/
    int robot_target = -1;             // aim
    int robot_id = 1;                  // 使用1号机器人实验
    auto &robot = meta.robot[robot_id];
    robot_target = next_target(-1);    // 初始化目标点
    while (cin.eof() == false)
    {
        cerr << "info: flame read" << endl;
        io::read_flame(cin);
        cerr << "info: flame read end, flame:" << meta.current_flame << endl;
        if (robot.in_station == robot_target)
        {
            robot_target = next_target(robot_target);
        }
        auto target = meta.station[robot_target].loc;
        navigate::move_to(robot_id, target);
        io::print_instructions(navigate::instructions, cout, meta.current_flame);
    }
    return 0;
}
