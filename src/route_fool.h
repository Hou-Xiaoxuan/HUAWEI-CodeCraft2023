#ifndef __ROUT_FOOL_H__
#define __ROUT_FOOL_H__
#include "model.h"
#include "navigate.h"
#include <algorithm>
#include <optional>
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

vector<int> processing;    // 机器人[i]正在处理的route
vector<int> processing_state;    // 机器人[i]正在处理的root的状态：0：取货中，1：运输中，2：卸货中
enum ProcessingState {
    PICKING = 0,
    BUY = 1,
    SELL = 2,
};
vector<double> super_demand;    // 是否有workstation正在等待货物[i]
struct {
    double bias = 0;
    int cnt = 0;
    inline double get() { return cnt > 0 ? bias / cnt : 0; }
} __estimated_bias;
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
    {
        for (int j = 1; j < meta.station.size(); j++)
        {
            const auto &from = meta.station[i], &to = meta.station[j];
            if (find(to.workstation().needs.begin(), to.workstation().needs.end(), from.product_id())
                != to.workstation().needs.end())
                add_edge(from, to);
        }
    }
    processing.assign(meta.robot.size(), 0);
    processing_state.assign(meta.robot.size(), ProcessingState::PICKING);
    super_demand.assign(model::goods.size(), 0);
}


/*TODO 参考navigate的实现，给出准确的移动时间帧数量预估*/
int __estimated_move_flame(Point from, Point target_one, Point target_two = Point(0, 0))
{
    const static double stable_bias = 10;
    double time = 0;
    time += Point::distance(from, target_one) / ConVar::max_robot_forward_speed;
    if (target_two.x != 0 and target_two.y != 0)
        time += Point::distance(target_one, target_two) / ConVar::max_robot_forward_speed;
    return static_cast<int>(time / 50.0) + stable_bias;
}

void __count_super_demand()
{
    /* 计算super_demand。如7在用于4后，会对5和6有强烈而需求
        demand在计算ddf是添加到利润中，激励生产高等级的原材料
        * 暂时不考虑不7即将得到4而不是已经得到哦啊的情况
    */
    super_demand.clear();
    super_demand.assign(model::goods.size(), 0);

    for (int i = 1; i < meta.station.size(); i++)
    {
        const auto &station = meta.station[i];
        if (station.material == 0) continue;
        if (station.workstation().needs.empty()) continue;

        vector<int> goods_true;
        vector<int> goods_false;
        for (int good : station.workstation().needs)
        {
            if (station.goods_exist(good))
                goods_true.push_back(good);
            else
                goods_false.push_back(good);
        }
        if (goods_false.empty()) continue;    // 该站点已经满足所有需求
        // 如果已经有货物往这里运送，就不要再增加需求了
        for (int j = 1; j < meta.robot.size(); j++)
        {
            if (processing[j] == 0) continue;
            const auto &route = routes[processing[j]];
            if (route.to_station_index == station.id)
                remove(goods_false.begin(), goods_false.end(), route.goods);
        }
        double demand_add = static_cast<double>(station.product().price - station.product().cost)
            * goods_true.size() / (goods_true.size() + goods_false.size()) * 0.35;
        for (auto good : goods_false)
            super_demand[good] += demand_add;
    }
}

int __give_pointing(int robot_id)
{
    __count_super_demand();

    const auto &robot = meta.robot[robot_id];
    int best_route_index = 0;
    double best_profit_per_flame = 0;
    int best_finish_time = -1;
    for (int i = 1; i < routes.size(); i++)
    {
        const auto &route = routes[i];
        /* 准入条件：
            1-1.1 终点暂时没有该货物
            1-1.2. 没有其他机器人在处理该任务/同级别任务(运送相同货物到相同终点站)
            1-2. 目标点是8、9号点(仅收购，无需等待完成）「1-1、1-2满足一点即可」
            2. 预期到达from点时，预期money足够
            3. 起点有货物/在生产(*优化可以加上可以预期得到货物*)
            4. 预期可以完成任务
            5. best_profit_per_flame最优
        *  expected_profit
            如果target已经有部分原材料/已有其他原材料在运输，则计算时要在profit中加入target生产货物利益的一部分
        *  expected_flame
            如果需要空转等待，则在计算是添加一部分惩罚
        */
        const auto &target_station = meta.station[route.to_station_index];
        const auto &from_station = meta.station[route.from_station_index];

        int expected_material = target_station.material;    // 预期到达from点时已有的原材料

        if (target_station.goods_exist(route.goods)) continue;                           // *contition 1-1.1
        if (from_station.with_product == 0 and from_station.timeleft == -1) continue;    // *condition 3

        int expected_money = meta.current_money;       // 预期到达from点时的money
        int expected_buy_flame = meta.current_flame    // 预期到达from点时的flame
            + __estimated_move_flame(robot.loc, from_station.loc);


        int invalid_route = 0;    // NOTE: invalid when [type not in (8&9) and 竞争终点] or [竞争起点]
        for (int j = 1; j < meta.robot.size(); j++)
        {

            if (processing[j] == 0) continue;
            if (robot_id == j) continue;

            const auto &p_route = routes[processing[j]];
            if (p_route.to_station_index == route.to_station_index)
            {
                if (p_route.goods == route.goods)    // 终点&货物相同
                    invalid_route += 1;              // *condition 1-1.2
                else
                {
                    expected_material &= (1 << p_route.goods);    // 添加预期原材料
                }
            }

            if (p_route.finish_time < expected_buy_flame and p_route.goods != 0)
                expected_money += model::goods[p_route.goods].price - model::goods[p_route.goods].cost;
            // 无法判断买货顺序实现先后，保险起见减去花费
            if (p_route.finish_time > expected_buy_flame and p_route.goods == 0)
                expected_money -= model::goods[route.goods].cost;


            // [等待时间计算] 竞争产物，退让
            if (p_route.from_station_index == route.from_station_index
                and meta.robot[j].goods == 0)    // 起点相同
                invalid_route = 100;
        }

        if (target_station.type == 8 || target_station.type == 9)
            invalid_route -= ConVar::max_robot;    // *condition 1-2
        if (invalid_route > 0) continue;

        if (expected_money < route.money_need) continue;    // *condition 2


        /* 计算profit per flame(ppf)的参数，可调参数：
           1.下一阶段收益的激励系数
           2. 等待时间的惩罚系数 */
        // [预期利润计算]：增加下一阶段预期产物的利润
        int expected_profit = route.profit;    // 预期利润
        if (target_station.type != 8 and target_station.type != 9)
            expected_profit += super_demand[target_station.product_id()];    // 更高阶段的预期利润

        if (target_station.type != 8 and target_station.type != 9)
        {
            int material_count = 1;    // expected_material的1的个数
            while (expected_material)
            {
                material_count += expected_material & 1;
                expected_material >>= 1;
            }
            expected_profit += (target_station.product().price - target_station.product().cost) * 0.5
                * material_count / target_station.product().needs.size();    // 加入预期利润0.5*原材料比例
        }

        int expected_flame_cost;
        if (from_station.with_product == 0
            and from_station.timeleft > __estimated_move_flame(robot.loc, from_station.loc))
        {
            expected_flame_cost
                = from_station.timeleft + __estimated_move_flame(from_station.loc, target_station.loc);
        }
        else
        {
            expected_flame_cost = __estimated_move_flame(robot.loc, from_station.loc, target_station.loc);
        }

        double flame_bias = 0;    // 帧数统计误差,时间越接近重点，越增大帧数误差

        if (meta.current_flame > 8000) flame_bias = __estimated_bias.get();
        if (meta.current_flame + expected_flame_cost + flame_bias > ConVar::time_limit)
            continue;    // *condition 4
        double ppf = expected_profit * 1.0 / expected_flame_cost;
        int empty_flame = 0;
        if (from_station.timeleft > 0)
            empty_flame
                = max((from_station.timeleft - __estimated_move_flame(robot.loc, from_station.loc)), 0);
        ppf -= empty_flame * 10;    // 空转惩罚，假设1000flame(20s)的预期收益是10000
        if (ppf > best_profit_per_flame)
        {
            best_profit_per_flame = ppf;
            best_route_index = i;
            best_finish_time = meta.current_flame + expected_flame_cost;
            cerr << "[info][__pointing] "
                 << " [flame=" << meta.current_flame << "] robot_id: " << robot_id
                 << "UPDATE best_profit_per_flame: " << best_profit_per_flame
                 << " best_route_index: " << best_route_index << " route: " << route << endl;
        }
    }

    if (best_route_index == 0)
    {
        cerr << "[info][pointing] robot " << robot_id << " no route" << endl;
    }
    else
    {
        cerr << "[info][__pointing] [flame=" << meta.current_flame << "] robot = " << robot_id
             << "best ppf = " << best_profit_per_flame << " route [" << best_route_index
             << "]: " << routes[best_route_index] << endl;
    }
    routes[best_route_index].finish_time = best_finish_time;
    return best_route_index;
}

/*1帧15ms内给出策略*/
vector<optional<Route>> give_pointing()
{
    for (int i = 1; i < meta.robot.size(); i++)
    {
        if (processing[i] == 0) processing[i] = __give_pointing(i);
        if (processing[i] == 0) continue;
        // 处理任务
        auto &route = routes[processing[i]];
        auto &robot = meta.robot[i];
        if (robot.goods == 0)
        {
            if (processing_state[i] == ProcessingState::SELL)    // 3->1
            {
                cerr << "[info][pointing] [flame=" << meta.current_flame << "] robot " << i << " finished"
                     << routes[processing[i]] << endl;

                __estimated_bias.bias += meta.current_flame - routes[processing[i]].finish_time;
                __estimated_bias.cnt++;
                routes[processing[i]].finish_time = -1;
                processing[i] = 0;
                processing_state[i] = ProcessingState::PICKING;
                continue;
            }
            processing_state[i] = ProcessingState::BUY;    // 1->2

            // TODO: 增加“不忠”逻辑，变更目标站点

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
            if (processing_state[i] == ProcessingState::BUY)    // 2->3
                processing_state[i] = ProcessingState::SELL;
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

    vector<optional<Route>> rt(meta.robot.size());

    for (int i = 1; i < meta.robot.size(); i++)
        if (processing[i] != 0) rt[i] = routes[processing[i]];
    return rt;
}
}

#endif