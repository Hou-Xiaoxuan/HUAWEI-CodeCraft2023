#ifndef _STOP_DIE_
#define _STOP_DIE_
#include "model.h"
#include "nav_model.h"
#include "nav_navigate.h"
#include <vector>
namespace stop_die
{
/* 特判机器人是否卡死，如果卡死，后退 */
using namespace std;
void stop_die()
{
    static bool is_init = false;
    static vector<int> back_until;             // 后退到哪一帧
    static vector<vector<int>> crash_frame;    // 机器人卡死的帧
    if (is_init == false)
    {
        is_init = true;
        back_until.assign(meta.robot.size(), -1);
        crash_frame.assign(meta.robot.size(), vector<int>(meta.robot.size(), 0));
    }

    /* 判断是否互相卡死 */
    for (int i = 1; i < meta.robot.size(); i++)
        for (int j = i + 1; j < meta.robot.size(); j++)
        {
            double dis = (meta.robot[i].goods > 0 ? ConVar::robot_radius_goods : ConVar::robot_radius)
                + (meta.robot[j].goods > 0 ? ConVar::robot_radius_goods : ConVar::robot_radius);
            if (Point::distance(meta.robot[i].loc, meta.robot[j].loc) <= dis)
            {
                crash_frame[i][j]++;
                cerr << "[info][stop_die] robot " << i << " and robot " << j << " crash "
                     << crash_frame[i][j] << endl;
            }
            else
                crash_frame[i][j] = 0;

            if (crash_frame[i][j] > 10)    // 卡死已有三帧
            {
                back_until[i] = meta.current_flame + 50;
                back_until[j] = meta.current_flame + 50;
            }
        }

    /* 执行后退逻辑 */
    for (int i = 1; i < meta.robot.size(); i++)
    {
        if (back_until[i] == -1) continue;

        if (meta.current_flame < back_until[i]) nav_navigate::move_back(meta.robot[i]);
    }
}
};
#endif