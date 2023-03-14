#ifndef __ROUT_FOOL_H__
#define __ROUT_FOOL_H__
#include "model.h"
#include "navigate.h"
#include <vector>
namespace route_fool
{
using namespace std;
struct Route {
    int from_station_index;
    int to_station_index;
    int finish_time;       // 预期任务完成时间，-1代表未开始
    int money_need;        // 开启任务需要资金
    int profit;            // 完成任务获得资金
    int profit_per_dis;    // 每单位距离获得资金

    const bool valid(int expected_money = meta.current_money) const
    {
        if (expected_money < money_need) return false;
    }
};
vector<Route> routes;
vector<vector<int>> point_to;    // 子节点，保存边的index
vector<vector<int>> point_by;    // 父节边，保存边的index

vector<int> processing;    // 机器人[i]正在处理的root
vector<int> processing_state;    // 机器人[i]正在处理的root的状态：0：取货中，1：运输中，2：卸货中
enum ProcessingState {
    PICKING = 0,
    BUY = 1,
    SELL = 2,    // 不会到达这个状态
};
/*4~5s内初始化所需要的变量*/
void init()
{
    routes.reserve(1000);
    point_by = vector<vector<int>>(meta.station.size());
    point_to = vector<vector<int>>(meta.station.size());
    routes.push_back(Route());    // 0号root为无效root

    // 根据goods的依赖关系建图
    auto first_class = [](int type) {
        return type == 1 || type == 2 || type == 3;
    };
    auto second_class = [](int type) {
        return type == 4 || type == 5 || type == 6;
    };
    auto add_edge = [&](const Station &from, const Station &to) {
        Route r;
        r.from_station_index = from.id;
        r.to_station_index = to.id;
        r.finish_time = -1;
        r.money_need = model::goods[from.product].cost;
        r.profit = (model::goods[from.product].price - model::goods[to.product].cost);
        r.profit_per_dis = (model::goods[from.product].price - model::goods[to.product].cost)
            / Point::distance(from.loc, to.loc);
        point_by[to.id].push_back(routes.size());
        point_to[from.id].push_back(routes.size());
        routes.push_back(r);
    };
    for (int i = 1; i < meta.station.size(); i++)
        for (int j = 1; j < meta.station.size(); j++)
        {
            const auto &from = meta.station[i], &to = meta.station[j];
            if (first_class(i) and second_class(j)) add_edge(from, to);
            if (second_class(i) and to.type == 7) add_edge(from, to);
            if (from.type == 7 and to.type == 8) add_edge(from, to);
            if (from.type != 9 and to.type == 9) add_edge(from, to);
        }
    processing.assign(meta.robot.size(), 0);
    processing.assign(meta.robot.size(), 0);
}


void __give_pointing(int robot_id) { }
/*1帧15ms内给出策略*/
void give_pointing()
{
    for (int i = 1; i < meta.robot.size(); i++)
    {
        if (processing[i] == 0)    // 分配任务
            __give_pointing(i);
        // 处理任务
        auto &route = routes[processing[i]];
        auto &robot = meta.robot[i];
        if (robot.goods == 0)
        {
            if (processing_state[i] == ProcessingState::SELL) /*结束了*/
            {
                processing[i] = 0;
                processing_state[i] = ProcessingState::PICKING;
                continue;
            }
            processing_state[i] = ProcessingState::BUY;
            if (robot.in_station == route.from_station_index)
            {
                io::instructions.push_back(new io::I_buy(i));
#ifdef DEBUG
                if (meta.station[route.from_station_index].timeleft > 0)
                {
                    cerr << "[error][pointing]"
                         << "robot " << i << " waiting for station " << route.from_station_index
                         << ", timeleft =" << meta.station[route.from_station_index].timeleft << endl;
                }
                if (meta.current_money < route.money_need)
                {
                    cerr << "[error][pointing]"
                         << "robot " << i << " waiting for money, need " << route.money_need << ", current "
                         << meta.current_money << endl;
                }
#endif
            }
            else
                navigate::move_to(i, meta.station[route.from_station_index].loc);
        }
        else
        {
            if (processing_state[i] == ProcessingState::BUY) processing_state[i] = ProcessingState::SELL;
            if (robot.in_station == route.to_station_index)
            {
                io::instructions.push_back(new io::I_sell(i));
#ifdef DEBUG
                if (meta.station[route.to_station_index].has_goods(robot.goods))
                    cerr << "[error][pointing]"
                         << "robot " << i << " station " << route.to_station_index << "already has goods "
                         << robot.goods << endl;
#endif
            }
            else
                navigate::move_to(i, meta.station[route.to_station_index].loc);
        }
    }
}
}

#endif