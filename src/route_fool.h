#ifndef __ROUT_FOOL_H__
#define __ROUT_FOOL_H__
#include "model.h"
#include "navigate.h"
#include <algorithm>
#include <vector>
namespace route_fool
{
using namespace std;
struct Route {
    int from_station_index;
    int to_station_index;
    int finish_time;    // 预期任务完成时间(帧)，-1代表未开始
    int money_need;     // 开启任务需要资金
    int profit;         // 完成任务获得资金
    int goods;          // 运输的货物
    friend ostream &operator<<(ostream &os, const Route &r)
    {
        os << "" << r.from_station_index << "(" << meta.station[r.from_station_index].type << ")"
           << "->" << r.to_station_index << "(" << meta.station[r.to_station_index].type << ")"
           << " finish_time:" << r.finish_time << " money_need:" << r.money_need;
        return os;
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
    SELL = 2,
};
/*4~5s内初始化所需要的变量*/
void init()
{
    routes.reserve(1000);
    point_by = vector<vector<int>>(meta.station.size());
    point_to = vector<vector<int>>(meta.station.size());
    routes.push_back(Route());    // 0号root为无效root

    // 根据goods的依赖关系建图
    auto add_edge = [&](const Station &from, const Station &to) {
        Route r;
        r.from_station_index = from.id;
        r.to_station_index = to.id;
        r.finish_time = -1;
        r.money_need = from.product().cost;
        r.profit = from.product().price - from.product().cost;
        r.goods = from.product_id();
        point_by[to.id].push_back(routes.size());
        point_to[from.id].push_back(routes.size());
        routes.push_back(r);
    };
    for (int i = 1; i < meta.station.size(); i++)
        for (int j = 1; j < meta.station.size(); j++)
        {
            const auto &from = meta.station[i], &to = meta.station[j];
            if (find(to.workstation().needs.begin(), to.workstation().needs.end(), from.product_id())
                != to.workstation().needs.end())
                add_edge(from, to);
        }
    processing.assign(meta.robot.size(), 0);
    processing_state.assign(meta.robot.size(), 0);
}


/*参考navigate的实现，给出准确的移动时间帧数量预估*/
int __estimated_move_flame(Point from, Point target_one, Point target_two = Point(0, 0))
{
    static const int bias = 3;    // 误差
    double time = 0;
    time += Point::distance(from, target_one) / ConVar::max_robot_forward_speed;
    if (target_two.x != 0 and target_two.y != 0)
        time += Point::distance(target_one, target_two) / ConVar::max_robot_forward_speed;
    return static_cast<int>(time / 50.0) + bias;
}
void __give_pointing(int robot_id)
{
    const auto &robot = meta.robot[robot_id];
    int best_route_index = -1, best_profit_per_flame = 0;
    for (int i = 1; i < routes.size(); i++)
    {
        const auto &route = routes[i];
        /* 准入条件：
            1-1.1 终点暂时没有该货物
            1-1.2. 没有其他机器人在处理该任务/同级别任务(运送相同货物到相同终点站)
            1-2. 目标点是8、9号点(仅收购，无需等待完成）「1-1、1-2满足一点即可」
            2. 预期到达from点时，预期money足够
            4. 预期可以完成任务
            5. progit_per_dis最优
        *  expected_profit
            -
        如果target已经有部分原材料/已有其他原材料在运输，则计算时要在profit中加入target生产货物利益的一部分
            -

        */
        const auto &target_station = meta.station[route.to_station_index];
        const auto &from_station = meta.station[route.from_station_index];

        int expected_material = target_station.material;    // 预期到达from点时已有的原材料
        if (target_station.goods_exist(route.goods) and target_station.type != 8    // *contition 1-1.1
            and target_station.type != 9)
        {
            // cerr << "[info][__pointing]"
            //      << " [flame=]" << meta.current_flame << " roobot_id: " << robot_id << "with type "
            //      << "skip route " << i << ": " << route << endl;
            continue;
        }

        int expecteed_money = meta.current_money;      // 预期到达from点时的money
        int expected_profit = route.profit;            // 预期利润
        int expected_buy_flame = meta.current_flame    // 预期到达from点时的flame
            + __estimated_move_flame(robot.loc, meta.station[routes[i].from_station_index].loc);

        // [等待时间计算]: 预期到达from点时，预期生产好材料所需要的时间
        int expected_ready_flame_count = ConVar::time_limit;    // from点预期生产好材料所需要的时间
        if (from_station.with_product == 1)
            expected_ready_flame_count = 0;
        else if (from_station.timeleft > 0)
            expected_ready_flame_count = min(expected_ready_flame_count, from_station.timeleft);
        else
            continue;    // 本论得不到货物则不再深入计算

        int invalid_route = 0;
        for (int j = 1; j < meta.robot.size(); j++)
        {

            if (processing[j] == 0) continue;
            auto &p_route = routes[processing[j]];
            if (p_route.to_station_index == route.to_station_index)
            {
                if (p_route.goods == route.goods)    // 终点&货物相同
                    invalid_route += 1;              // *condition 1-1.2
                else
                {
                    expected_material &= (1 << (p_route.goods - 1));    // 添加预期原材料
                }
            }
            // expected_money
            if (routes[processing[j]].finish_time <= expected_buy_flame)
                expecteed_money += routes[processing[j]].money_need + routes[processing[j]].profit;

            // [等待时间计算] 竞争产物，退让
            if (p_route.from_station_index == route.from_station_index
                and meta.robot[j].goods == 0)    // 起点相同
                invalid_route = 100;
        }
        // cerr << "[info][__pointing]"
        //      << " [flame=]" << meta.current_flame << " roobot_id: " << robot_id
        //      << "with invalid= " << invalid_route << "skip route " << i << ": " << route << endl;
        if (target_station.type == 8 || target_station.type == 9)
            invalid_route -= ConVar::max_robot;    // *condition 1-2
        if (invalid_route > 0) continue;
        if (expecteed_money < route.money_need) continue;    // *condition 2

        // [预期利润计算]：增加下一阶段预期产物的利润
        if (target_station.type == 8 or target_station.type == 9)
        { }    // 啥也不做
        else
        {
            // expected_material的1的个数
            int material_count = 0;
            while (expected_material)
            {
                material_count += expected_material & 1;
                expected_material >>= 1;
            }
            expected_profit += (target_station.product().cost - target_station.product().price) * 0.5
                * material_count / target_station.product().needs.size();    // 加入预期利润0.5*原材料比例
        }

        // 计算利润率
        // int expected_flame_cost = max(expected_ready_flame_count,
        //     __estimated_move_flame(robot.loc, meta.station[route.from_station_index].loc));
        // expected_flame_cost += __estimated_move_flame(
        //     meta.station[route.from_station_index].loc, meta.station[route.to_station_index].loc);
        int expected_flame_cost;
        if (expected_buy_flame > __estimated_move_flame(robot.loc, from_station.loc))
        {
            expected_flame_cost
                = expected_buy_flame + __estimated_move_flame(from_station.loc, target_station.loc);
        }
        else
        {
            expected_flame_cost = __estimated_move_flame(robot.loc, from_station.loc, target_station.loc);
        }

        // cerr << "[info][__pointing]"
        //      << " [flame=]" << meta.current_flame << " robot_id: " << robot_id
        //      << " expected_flame_cost: " << expected_flame_cost << " expected_profit: " <<
        //      expected_profit
        //      << endl;
        if (meta.current_flame + expected_flame_cost > ConVar::time_limit) continue;    // *condition 4
        if (expected_profit / expected_flame_cost > best_profit_per_flame)
        {
            best_profit_per_flame = expected_profit / expected_flame_cost;
            best_route_index = i;
            cerr << "[info][__pointing] "
                 << " [flame=" << meta.current_flame << "] robot_id: " << robot_id
                 << "UPDATE best_profit_per_flame: " << best_profit_per_flame
                 << " best_route_index: " << best_route_index << " route: " << route << endl;
        }
    }

    if (best_route_index == -1)
    {
        cerr << "[info][pointing] robot " << robot_id << " no route" << endl;
    }
    else
    {
        cerr << "[info][__pointing] [flame=" << meta.current_flame << "] robot = " << robot_id
             << "best ppf = " << best_profit_per_flame << " route [" << best_route_index
             << "]: " << routes[best_route_index] << endl;
        processing[robot_id] = best_route_index;
        processing_state[robot_id] = ProcessingState::PICKING;
    }
}
/*1帧15ms内给出策略*/
void give_pointing()
{
    for (int i = 1; i < meta.robot.size(); i++)
    {
        if (processing[i] == 0)    // 分配任务
            __give_pointing(i);
        if (processing[i] == 0) continue;
        // 处理任务
        auto &route = routes[processing[i]];
        auto &robot = meta.robot[i];
        if (robot.goods == 0)
        {
            if (processing_state[i] == ProcessingState::SELL) /*结束了*/
            {
                cerr << "[info][pointing] [flame=" << meta.current_flame << "] robot " << i << " finished"
                     << routes[processing[i]] << endl;
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
                navigate::move_to(robot, meta.station[route.from_station_index].loc);
        }
        else
        {
            if (processing_state[i] == ProcessingState::BUY) processing_state[i] = ProcessingState::SELL;
            if (robot.in_station == route.to_station_index)
            {
                io::instructions.push_back(new io::I_sell(i));
#ifdef DEBUG
                if (meta.station[route.to_station_index].goods_exist(robot.goods))
                    cerr << "[error][pointing]"
                         << "robot " << i << " station " << route.to_station_index << "already has goods "
                         << robot.goods << endl;
#endif
            }
            else
                navigate::move_to(robot, meta.station[route.to_station_index].loc);
        }
    }
}
}

#endif