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
    int money_need;    // 开启任务需要资金
    int profit;        // 完成任务获得资金
    int goods;         // 运输的货物

    // 状态值
    int start_flame;     // 预期任务开始时间(帧)，-1代表未开始
    int finish_flame;    // 预期任务完成时间(帧)，-1代表未开始
    double ppf;          // profit per frame
    friend ostream &operator<<(ostream &os, const Route &r)
    {
        os << "" << r.from_station_index << "(" << meta.station[r.from_station_index].type << ")"
           << "@" << meta.station[r.from_station_index].loc << "->" << r.to_station_index << "("
           << meta.station[r.to_station_index].type << ")"
           << "@" << meta.station[r.to_station_index].loc;
        return os;
    }
    inline void clear_state()
    {
        start_flame = -1;
        finish_flame = -1;
        ppf = 0;
    }
};


vector<Route> routes;
vector<vector<size_t>> point_to;    // 子节点，保存边的index
vector<vector<size_t>> point_by;    // 父节边，保存边的index

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
    inline void add(double v)
    {
        bias += v;
        cnt++;
    }
    inline double get() { return cnt > 0 ? bias / cnt : 0; }
} _estimated_bias;
/*4~5s内初始化所需要的变量*/
void init()
{
    routes.reserve(1000);
    point_by.assign(meta.station.size(), vector<size_t>());
    point_to.assign(meta.station.size(), vector<size_t>());
    routes.emplace_back();    // 0号root为无效root

    // 根据goods的依赖关系建图
    auto add_edge = [&](const Station &from, const Station &to) {
        Route r;
        r.from_station_index = from.id;
        r.to_station_index = to.id;
        r.finish_flame = -1;
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
int _estimated_move_flame(Point from, Point target_one, Point target_two = Point(0, 0))
{
    const static double stable_bias = 15;
    double time = 0;
    time += Point::distance(from, target_one) / ConVar::max_robot_forward_speed;
    if (target_two.x != 0 and target_two.y != 0)
        time += Point::distance(target_one, target_two) / ConVar::max_robot_forward_speed;
    return static_cast<int>(time * 50.0 + stable_bias);
}

/* 全局统计demand需求 */
void _count_super_demand()
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
        }
        double demand_add = static_cast<double>(station.product().price - station.product().cost)
            * static_cast<double>(goods_true.size())
            / static_cast<double>(goods_true.size() + goods_false.size()) * 0.35;
        for (auto good : goods_false)
            super_demand[good] += demand_add;
    }
}

/* 商品价值衰减率 */
double decrease_factor(int x, int maxX, double minRate = 0.8)
{
    if (x < maxX) return (1 - sqrt(1 - pow((1 - static_cast<double>(x) / maxX), 2))) + minRate;
    return minRate;
}

double _get_expected_profit(const Robot &robot, const Route &route)
{
    const auto &from_station = meta.station[route.from_station_index];
    const auto &target_station = meta.station[route.to_station_index];

    int expected_material = target_station.material;
    for (int j = 1; j < meta.robot.size(); j++)
    {
        if (processing[j] == 0) continue;
        if (robot.id == j) continue;
        const auto &p_route = routes[processing[j]];
        if (p_route.to_station_index == route.to_station_index)
        {
            if (p_route.goods != route.goods)
                expected_material &= (1 << p_route.goods);    // 添加预期原材料
        }
    }

    auto expected_profit = static_cast<double>(route.profit);    // 预期利润
    // expected_profit *= decrease_factor(_estimated_move_flame(from_station.loc, target_station.loc),
    //     ConVar::time_limit);
    if (target_station.workstation().is_consumer() == false)
        expected_profit += super_demand[target_station.product_id()];    // 更高阶段的预期利润

    if (target_station.workstation().is_consumer() == false)
    {
        int material_count = 1;    // expected_material的1的个数
        while (expected_material)
        {
            material_count += expected_material & 1;
            expected_material >>= 1;
        }
        expected_profit
            += static_cast<double>(target_station.product().price - target_station.product().cost) * 0.5
            * material_count
            / static_cast<double>(target_station.product().needs.size());    // 加入预期利润0.5*原材料比例
    }

    int empty_flame = 0;
    if (from_station.timeleft > 0)
        empty_flame = max((from_station.timeleft - _estimated_move_flame(robot.loc, from_station.loc)), 0);
    expected_profit -= empty_flame * 10;    // 空转惩罚，假设1000flame(20s)的预期收益是10000

    return expected_profit;
}

int _get_expected_flame_cost(const Robot &robot, const Route &route)
{
    const auto &from_station = meta.station[route.from_station_index];
    const auto &target_station = meta.station[route.to_station_index];
    int expected_flame_cost;
    if (from_station.with_product == 0
        and from_station.timeleft > _estimated_move_flame(robot.loc, from_station.loc))
    {
        expected_flame_cost
            = from_station.timeleft + _estimated_move_flame(from_station.loc, target_station.loc);
    }
    else
    {
        expected_flame_cost = _estimated_move_flame(robot.loc, from_station.loc, target_station.loc);
    }
    return expected_flame_cost;
}

/* 抢夺其他机器人任务 */
int _steal_pointing(int robot_id)
{
    const auto &robot = meta.robot[robot_id];
    for (const auto &p_robot : meta.robot)
    {
        if (p_robot.id == meta.robot.begin()->id) continue;
        if (processing[p_robot.id] == 0) continue;
        if (p_robot.id == robot_id) continue;
        if (p_robot.goods != 0) continue;
        cerr << "[info][__steal_pointing] p_robot.id=" << p_robot.id << endl;

        auto &route = routes[processing[p_robot.id]];

        // XXX 简单策略，距离更近
        if (_estimated_move_flame(robot.loc, meta.station[route.from_station_index].loc) * 2
            < _estimated_move_flame(p_robot.loc, meta.station[route.from_station_index].loc))
        {
            // 成功抢夺
            processing[robot_id] = processing[p_robot.id];
            processing[p_robot.id] = 0;
            // 信息更新
            double expected_profit = _get_expected_profit(robot, route);
            int expected_flame_cost = _get_expected_flame_cost(robot, route);
            double ppf = expected_profit / expected_flame_cost;
            route.ppf = ppf;
            route.start_flame = meta.current_flame;
            route.finish_flame = meta.current_flame + expected_flame_cost;
            return processing[robot_id];
        }
    }
    return 0;
}

/* 分配任务 */
int _give_pointing(int robot_id, double init_ppf = 0.0)
{
    _count_super_demand();
    const auto &robot = meta.robot[robot_id];
    struct {
        int index = 0;
        int finish_flame = 0;
        double ppf = -1;
    } best_route;
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

        if (target_station.goods_exist(route.goods)) continue;                           // *contition 1-1.1
        if (from_station.with_product == 0 and from_station.timeleft == -1) continue;    // *condition 3

        int expected_money = meta.current_money;       // 预期到达from点时的money
        int expected_buy_flame = meta.current_flame    // 预期到达from点时的flame
            + _estimated_move_flame(robot.loc, from_station.loc);


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
            }

            if (p_route.finish_flame < expected_buy_flame and p_route.goods != 0)
                expected_money += model::goods[p_route.goods].price - model::goods[p_route.goods].cost;
            // 无法判断买货顺序实现先后，保险起见减去花费
            if (p_route.finish_flame > expected_buy_flame and p_route.goods == 0)
                expected_money -= model::goods[route.goods].cost;


            // [等待时间计算] 竞争产物，退让
            if (p_route.from_station_index == route.from_station_index
                and meta.robot[j].goods == 0)    // 起点相同
                invalid_route = 100;

            // [就近原则]
            if (route.from_station_index == p_route.to_station_index) invalid_route = 100;
        }

        if (target_station.workstation().is_consumer())
            invalid_route -= ConVar::max_robot;    // *condition 1-2
        if (invalid_route > 0) continue;

        if (expected_money < route.money_need) continue;    // *condition 2


        /*计算、选择最佳ppf*/
        double expected_profit = _get_expected_profit(robot, route);         // 预期利润
        int expected_flame_cost = _get_expected_flame_cost(robot, route);    // 预期时间

        double flame_bias = 0;    // 帧数统计误差,时间越接近重点，越增大帧数误差
        if (meta.current_flame > 8000) flame_bias = _estimated_bias.get();
        if (meta.current_flame + expected_flame_cost + flame_bias > ConVar::time_limit)
            continue;    // *condition 4

        double ppf = expected_profit / expected_flame_cost;
#ifdef DEBUG
        if (ppf > best_route.ppf)
        {
            best_route = {i, meta.current_flame + expected_flame_cost, ppf};
            cerr << "[info][__pointing] "
                 << " [flame=" << meta.current_flame << "] robot_id: " << robot_id
                 << " UPDATE best_profit_per_flame: " << best_route.ppf << " profit: " << expected_profit
                 << " flame_cost: " << expected_flame_cost << " best_route_index: " << best_route.index
                 << " route: " << route << endl;
        }
        else
        {
            cerr << "[info][__pointing] [flame=" << meta.current_flame << "] robot_id=" << robot_id
                 << " profit=" << expected_profit << " flame_cost=" << expected_flame_cost
                 << " route=" << route << " valid, but ppf=" << ppf << endl;
        }
#else
        if (ppf > best_route.ppf) best_route = {i, meta.current_flame + expected_flame_cost, ppf};
#endif
    }
#ifdef DEBUG
    if (best_route.index == 0)
    {
        cerr << "[info][pointing] robot " << robot_id << " no route" << endl;
    }
    else
    {
        cerr << "[info][__pointing] [flame=" << meta.current_flame << "] robot = " << robot_id
             << "best ppf = " << best_route.ppf << " route [" << best_route.index
             << "]: " << routes[best_route.index] << endl;
    }
#endif
    routes[best_route.index].start_flame = meta.current_flame;
    routes[best_route.index].finish_flame = best_route.finish_flame;
    routes[best_route.index].ppf = best_route.ppf;

    return best_route.index;
}

/*1帧15ms内给出策略*/
vector<optional<Route>> give_pointing()
{
    for (int i = 1; i < meta.robot.size(); i++)
    {
        // if (processing[i] == 0) processing[i] = __steal_pointing(i); // 负优化
        if (processing[i] == 0) processing[i] = _give_pointing(i);
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

                _estimated_bias.add(meta.current_flame - routes[processing[i]].finish_flame);
                routes[processing[i]].clear_state();
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