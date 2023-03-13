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
    while (cin.eof() == false)
    {
        cerr << "info: flame read" << endl;
        io::read_flame(cin);
        cerr << "info: flame read end, flame:" << meta.current_flame << endl;

        vector<int> robot_target(5, -1);    // 机器人目标点
        int robot_id = 1;                   // 使用1号机器人实验
        if (robot_target[robot_id] == -1) robot_target[robot_id] = next_target(-1);    // 初始化目标点
        auto target = meta.station[robot_target[robot_id]].loc;
        navigate::move_to(robot_id, target);
        io::print_instructions(navigate::instructions, cout, meta.current_flame);
    }
    return 0;
}
