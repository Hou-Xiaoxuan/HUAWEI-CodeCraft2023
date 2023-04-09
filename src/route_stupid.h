/** 复赛路径分配算法，修改自route_fool.h
 */
#ifndef ROUTE_STUPID_H
#define ROUTE_STUPID_H
#include "args.h"
#include "const.h"
#include "iointerface.h"
#include "model.h"
#include "nav_model.h"
#include "nav_navigate.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>

namespace route_stupid
{
using namespace std;
using find_path_square::find_path;
using find_path_square::find_shelter_path;

int _estimated_move_flame(const vector<navmesh::Vertex> &path)
{
    double dis = 0;
    for (size_t i = 1; i < path.size(); i++)
        dis += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    dis *= Args::distance_factor;
    return static_cast<int>(dis / ConVar::max_robot_forward_speed) * 50
        + Args::turn_cost * static_cast<int>(path.size());
}
using Path = vector<navmesh::Vertex>;
/*Route与road_pair的区别
- Rotue是有合法供应关系的工作站之间的节点对
- road_pair是任意工作站之间的节点对
*/
struct _road_pair {
    int from;    // 起点工作站编号
    int to;      // 终点工作站编号
    bool operator==(const _road_pair &p) const { return from == p.from && to == p.to; }
};
struct _roud_pair_hash {
    size_t operator()(const _road_pair &p) const { return p.from * 100 + p.to; }
};
vector<vector<Path>> pathes;    // 路径缓存
struct Route {
    int from_station_index;      // 起点工作台编号
    int target_station_index;    // 终点工作台编号
    const Path &path() const { return pathes.at(from_station_index).at(target_station_index); }
    // 起点工作台
    const Station &from_station() const { return meta.station[from_station_index]; }
    // 终点工作台
    const Station &target_station() const { return meta.station[target_station_index]; }
    // 运送货物
    const Goods &goods() const { return this->from_station().product(); }
    // 运输距离
    double profit() const { return this->goods().price - this->goods().cost; }

    friend ostream &operator<<(ostream &os, const Route &r)
    {
        os << "Route { from: " << r.from_station_index << "(" << r.from_station().type << ")"
           << ", to: " << r.target_station_index << "(" << r.target_station().type << ")"
           << " }";
        return os;
    }
};

/*机器人状态量*/
enum ProcessingState {
    PICKING = 0,    // 取货中
    BUY = 1,        // 运输中
    SELL = 2,       // 卖货中
};
static vector<int> processing {};                      // 机器人[i]正在处理的route
static vector<ProcessingState> processing_state {};    // 机器人[i]正在处理的root的状态
static vector<int> stop_until {};    // 机器人停止获取pointing的标志，缓解跳帧
/*区域划分与算法*/
class Area
{
public:
private:
    vector<Route> routes;           // 区域内的路径, 从1开始
    set<int> stations;              // 区域内的工作台
    vector<double> super_demand;    // 是否有workstation正在等待货物[i]
private:
    void _update_super_demand()
    {
        this->super_demand.clear();
        this->super_demand.assign(model::goods.size(), 0);

        for (const auto station_id : this->stations)
        {
            const auto &station = meta.station[station_id];
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
                / static_cast<double>(goods_true.size() + goods_false.size()) * Args::super_demand_ratio;
            for (auto good : goods_false)
                super_demand[good] += demand_add;
        }
    }

    double _get_expected_profit(const Robot &robot,
        const Route &route,
        int now_station,
        const Path &now_station_path)
    {
        const auto &from_station = route.from_station();
        const auto &target_station = route.target_station();
        int expected_material = target_station.material;
        for (int j = 1; j < meta.robot.size(); j++)
        {
            if (processing[j] == 0) continue;
            if (robot.id == j) continue;
            const auto &p_route = routes[processing[j]];
            if (p_route.target_station_index == route.target_station_index)
            {
                if (p_route.goods().type != route.goods().type)
                    expected_material &= (1 << p_route.goods().type);    // 添加预期原材料
            }
        }

        auto expected_profit = static_cast<double>(route.profit());    // 预期利润

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
            auto profit_deeper
                = static_cast<double>(target_station.product().price - target_station.product().cost)
                * Args::deeper_profit_ratio * material_count
                / static_cast<double>(target_station.product().needs.size());
            expected_profit += profit_deeper;
        }

        int empty_flame = 0;
        if (from_station.timeleft > 0)
        {

            vector<navmesh::Vertex> path;
            if (now_station <= 0)
            {
                if (_USE_LOG_)
                {
                    // robot.loc->from_station.loc
                    cerr << "[warning][get_expected_profit] robot[" << robot.id
                         << "] didn't have now station path" << endl;
                }
                path = find_path(robot.loc, from_station.loc, robot.goods > 0);
            }
            else
                path = pathes[now_station][from_station.id];

            // empty_flame
            //     = max((from_station.timeleft - _estimated_move_flame(robot.loc, from_station.loc)), 0);
            empty_flame = max((from_station.timeleft - _estimated_move_flame(path)
                                  - _estimated_move_flame(now_station_path)),
                0);
        }
        expected_profit
            -= empty_flame * Args::wait_blame;    // 空转惩罚，假设1000flame(20s)的预期收益是10000
        return expected_profit;
    }

    int _get_expected_flame_cost(const Robot &robot,
        const Route &route,
        int station_now,
        const Path &now_station_path)
    {
        const auto &from_station = route.from_station();
        const auto &target_station = route.target_station();
        int expected_flame_cost;
        vector<navmesh::Vertex> robot_to_from;
        vector<navmesh::Vertex> from_to_target;
        if (station_now <= 0)
        {
            robot_to_from = find_path(robot.loc, from_station.loc, false);
            if (_USE_LOG_)
            {
                cerr << "[warning][get_expected_flame_cost] robot[" << robot.id
                     << "] is not start from any station" << endl;
            }
        }
        else
            robot_to_from = pathes.at(station_now).at(from_station.id);
        from_to_target = pathes.at(from_station.id).at(target_station.id);
        if (from_station.with_product == 0
            and from_station.timeleft
                > _estimated_move_flame(robot_to_from) + _estimated_move_flame(now_station_path))
            expected_flame_cost = from_station.timeleft + _estimated_move_flame(from_to_target);
        else
            expected_flame_cost = _estimated_move_flame(robot_to_from)
                + _estimated_move_flame(from_to_target) + _estimated_move_flame(now_station_path);
        return expected_flame_cost;
    }


public:
    Area() : super_demand(meta.station.size()) { }
    friend void init();
    friend void give_pointing();
    int _give_pointing(int robot_id, double init_ppf = 0.0)
    {
        this->_update_super_demand();
        const auto &robot = meta.robot[robot_id];
        int now_station = robot.in_station;
        Path now_station_path {};
        if (now_station <= 0)
        {
            now_station_path = find_path_square::find_nearest_workshop(robot.loc);
            if (now_station_path.size() > 0u)
            {
                const auto &station_loc = now_station_path.back();
                for (auto station_index : this->stations)
                {
                    if (std::hypot(station_loc.x - meta.station[station_index].loc.x,
                            station_loc.y - meta.station[station_index].loc.y)
                        < navmesh::EPS)
                    {
                        now_station = station_index;
                        if (_USE_LOG_)
                        {
                            cerr << "[info][give_pointing] robot[" << robot_id
                                 << "] is not in any station, but find now station [" << now_station
                                 << "], path size: " << now_station_path.size() << endl;
                        }
                        break;
                    }
                }
            }
        }
        struct {
            int index = 0;
            int finish_flame = 0;
            double ppf = -1;
        } best_route;
        for (int i = 1; i < this->routes.size(); i++)
        {
            const auto &route = routes[i];
            const auto &from_station = route.from_station();
            const auto &target_station = route.target_station();

            if (target_station.goods_exist(route.goods().type)) continue;    // *contition 1-1.1
            if (from_station.with_product == 0 and from_station.timeleft == -1) continue;    // *condition 3

            int invalid_route = 0;    // NOTE: invalid when [type not in (8&9) and 竞争终点] or [竞争起点]
            for (int j = 1; j < meta.robot.size(); j++)
            {

                if (processing[j] == 0) continue;
                if (robot_id == j) continue;

                const auto &p_route = routes[processing[j]];
                if (p_route.target_station_index == route.target_station_index)
                {
                    if (p_route.goods().type == route.goods().type)    // 终点&货物相同
                        invalid_route += 1;                            // *condition 1-1.2
                }

                // [等待时间计算] 竞争产物，退让
                if (p_route.from_station_index == route.from_station_index
                    and meta.robot[j].goods == 0)    // 起点相同
                    invalid_route = 100;

                // [就近原则]
                if (route.from_station_index == p_route.target_station_index)    // *condition 6
                    invalid_route = 100;
            }

            if (target_station.workstation().is_consumer())
                invalid_route -= ConVar::max_robot;    // *condition 1-2
            if (invalid_route > 0) continue;


            /*计算、选择最佳ppf*/
            double expected_profit
                = _get_expected_profit(robot, route, now_station, now_station_path);    // 预期利润
            int expected_flame_cost
                = _get_expected_flame_cost(robot, route, now_station, now_station_path);    // 预期时间

            if (meta.current_flame + expected_flame_cost > ConVar::time_limit)    // *condition 4
            {
                if (_USE_LOG_)
                {
                    cerr << "[info][give_pointing] frame" << meta.current_flame << " robot[" << robot_id
                         << "] route[" << i << "] flame_cost[" << expected_flame_cost << "] > time_limit["
                         << ConVar::time_limit << "]" << endl;
                }
                continue;
            }

            double ppf = expected_profit / expected_flame_cost;

            if (_USE_LOG_)
            {
                if (ppf > best_route.ppf)
                {
                    best_route = {i, meta.current_flame + expected_flame_cost, ppf};
                    cerr << "[info][__pointing] "
                         << " [flame=" << meta.current_flame << "] robot_id: " << robot_id
                         << " UPDATE best_profit_per_flame: " << best_route.ppf
                         << " profit: " << expected_profit << " flame_cost: " << expected_flame_cost
                         << " best_route_index: " << best_route.index << " route: " << route << endl;
                }
                else
                {
                    cerr << "[info][__pointing] [flame=" << meta.current_flame << "] robot_id=" << robot_id
                         << " profit=" << expected_profit << " flame_cost=" << expected_flame_cost
                         << " route=" << route << " valid, but ppf=" << ppf << "with profit"
                         << expected_profit << " flame_cost" << expected_flame_cost
                         << " is not better than best_route.ppf=" << best_route.ppf << endl;
                }
            }

            if (ppf > best_route.ppf) best_route = {i, meta.current_flame + expected_flame_cost, ppf};
        }

        // #ifdef DEBUG
        //         if (best_route.index == 0)
        //         {
        //             cerr << "[info][pointing] robot " << robot_id << " no route" << endl;
        //         }
        //         else
        //         {
        //             cerr << "[info][__pointing] [flame=" << meta.current_flame << "] robot = " <<
        //             robot_id
        //                  << "best ppf = " << best_route.ppf << " route [" << best_route.index
        //                  << "]: " << routes[best_route.index] << endl;
        //         }
        // #endif
        // routes[best_route.index].start_flame = meta.current_flame;
        // routes[best_route.index].finish_flame = best_route.finish_flame;
        // routes[best_route.index].ppf = best_route.ppf;
        if (_USE_LOG_)
        {
            cerr << "[info][__pointing] [flame=" << meta.current_flame << "] robot = " << robot_id
                 << "best ppf = " << best_route.ppf << " route [" << best_route.index
                 << "]: " << this->routes[best_route.index] << endl;
        }
        if (best_route.index == 0)
        {
            if (_USE_LOG_)
            {
                cerr << "[error][__pointing] best_route.index == 0" << endl;
            }
        }
        return best_route.index;
    }
};

vector<Area> areas;    // 区域，从1开始
void init()
{
#ifdef DEBUG
    for (auto &sta : model::meta.station)
    {
        if (sta.workstation().type <= 3) sta.with_product = 1;
    }
#endif

    vector<Route> routes;    // 路径，从1开始
    pathes = vector<vector<Path>>(meta.station.size(), vector<Path>(meta.station.size(), Path()));
    routes.reserve(1000);
    routes.emplace_back();

    // 并查集
    vector<int> fa(meta.station.size());
    for (int i = 1; i < meta.station.size(); i++)
        fa[i] = i;
    function<int(int)> find_fa;
    find_fa = [&fa, &find_fa](int x) -> int {
        if (fa[x] == x) return x;
        return fa[x] = find_fa(fa[x]);
    };
    auto merge = [&fa, &find_fa](int x, int y) {
        x = find_fa(x);
        y = find_fa(y);
        if (x == y) return;
        fa[x] = y;
    };


    for (int i = 1; i < meta.station.size(); i++)
        for (int j = 1; j < meta.station.size(); j++)
        {
            const Station &from = meta.station[i], target = meta.station[j];
            vector<navmesh::Vertex> path = find_path(from.loc, target.loc, true);
            if (path.empty()) continue;
            // 没有供应关系也要建边
            merge(i, j);
            pathes[i][j] = path;
            pathes[j][i] = path;

            if (std::find(
                    target.workstation().needs.begin(), target.workstation().needs.end(), from.product_id())
                == target.workstation().needs.end())
                continue;    // 不通
            routes.emplace_back(Route {i, j});
        }

    // 按照机器人划分为4个区域(有重叠)
    areas.assign(meta.robot.size(), {});
    for (int i = 1; i < meta.robot.size(); i++)
    {
        areas[i].routes.emplace_back();
        areas[i].routes.reserve(100);
    }
    for (int i = 1; i < meta.station.size(); i++)
    {
        int x = find_fa(i);
        if (x == i)
        {
            Area tmp_area;
            tmp_area.routes.reserve(100);
            // 将包含x的所有route加入sub_area
            for (int j = 1; j < routes.size(); j++)
                if (x == find_fa(routes[j].from_station_index)
                    or x == find_fa(routes[j].target_station_index))
                {
                    tmp_area.routes.emplace_back(routes[j]);
                    tmp_area.stations.insert(routes[j].from_station_index);
                    tmp_area.stations.insert(routes[j].target_station_index);
                }
            if (tmp_area.routes.size() > 0)
            {
                for (int k = 1; k < meta.robot.size(); k++)
                {
                    auto path = find_path(meta.robot[k].loc, meta.station[i].loc, false);
                    if (path.empty())
                    {
                        if (_USE_LOG_)
                        {
                            cerr << "[waring][route_stupid init] robot " << k
                                 << " can't reach station class " << i << endl;
                        }
                        continue;
                    }

                    // 合并
                    for (auto &route : tmp_area.routes)
                    {
                        areas[k].routes.emplace_back(route);
                        if (_USE_LOG_)
                        {
                            cerr << "[info][init] area " << k << " add route " << route << endl;
                        }
                    }
                    areas[k].stations.insert(tmp_area.stations.begin(), tmp_area.stations.end());
                }
            }
        }
    }
    // debug

    if (_USE_LOG_)
    {

        for (int i = 1; i < areas.size(); i++)
        {
            cerr << "[info][init] area " << i << " routes size = " << areas[i].routes.size()
                 << "station size" << areas[i].stations.size() << endl;

            cerr << endl;
        }
    }

    // enddebug
    processing.assign(meta.robot.size(), 0);
    processing_state.assign(meta.robot.size(), ProcessingState::PICKING);
    stop_until.assign(meta.robot.size(), -1);
}
///////////////////////////////////////////////////////////////////////////////////
// void process_anticollision(vector<Path> &robot_path)
// {
//     // 处理，改变robot_path
//     static vector<optional<navmesh::Vertex>> shelter_vertex;
//     if (shelter_vertex.empty()) shelter_vertex.assign(meta.robot.size(), nullopt);
//     for (int i = 1; i < meta.robot.size(); i++)
//     {
//         if (shelter_vertex[i].has_value())
//         {
//             if (hypot(
//                     meta.robot[i].loc.x - shelter_vertex[i]->x, meta.robot[i].loc.y -
//                     shelter_vertex[i]->y)
//                 < 0.2)
//                 shelter_vertex[i] = nullopt;
//             else
//             {
//                 robot_path[i] = find_path(meta.robot[i].loc, *shelter_vertex[i], meta.robot[i].goods >
//                 0.5);
//             }
//         }
//     }

//     for (int pri = 1; pri < robot_path.size(); ++pri)
//     {
//         for (int sub = pri + 1; sub < robot_path.size(); ++sub)
//         {
//             const auto &pri_path = robot_path[pri];
//             const auto &sub_path = robot_path[sub];
//             bool need_shelter = false;
//             double left_dis_pri = 3.0;
//             double left_dis_sub = 3.0;

//             for (int i = 0; i < pri_path.size(); ++i)
//             {
//                 navmesh::Vec2 vec_pri {pri_path[i], pri_path[(i + 1) % pri_path.size()]};
//                 navmesh::Segment pri_seg;
//                 // 过长截取
//                 if (vec_pri.length() >= left_dis_pri)
//                 {
//                     pri_seg = {
//                         pri_path[i],
//                         {pri_path[i].x + vec_pri.x * left_dis_pri / vec_pri.length(),
//                              pri_path[i].y + vec_pri.y * left_dis_pri / vec_pri.length()}
//                     };
//                 }
//                 else
//                 {
//                     pri_seg = {pri_path[i], pri_path[(i + 1) % pri_path.size()]};
//                 }
//                 left_dis_pri -= pri_seg.length();

//                 for (int j = 0; j < sub_path.size(); ++j)
//                 {
//                     // 过长截取
//                     navmesh::Vec2 vec_sub {sub_path[j], sub_path[(j + 1) % sub_path.size()]};
//                     navmesh::Segment sub_seg;
//                     if (vec_sub.length() >= left_dis_sub)
//                     {
//                         sub_seg = {
//                             sub_path[j],
//                             {sub_path[j].x + vec_sub.x * left_dis_sub / vec_sub.length(),
//                                  sub_path[j].y + vec_sub.y * left_dis_sub / vec_sub.length()}
//                         };
//                     }
//                     else
//                     {
//                         sub_seg = {sub_path[j], sub_path[(j + 1) % sub_path.size()]};
//                     }
//                     left_dis_sub -= sub_seg.length();

//                     double dis = navmesh::Segment::distance(pri_seg, sub_seg);
//                     if (dis <= 1.2) need_shelter = true;

//                     if (left_dis_sub <= 1e-6 or need_shelter) break;
//                 }
//                 if (left_dis_pri <= 1e-6 or need_shelter) break;
//             }

//             if (need_shelter)
//             {
//                 cerr << "[info][shelter] robot " << sub << " need shelter " << pri << endl;
//                 // cerr << "[debug][shelter]robot " << sub << " loc in " << sub_path[0] << " robot " <<
//                 pri
//                 //      << " path(" << pri_path.size() << "):"
//                 //      << " {";
//                 // print_path(robot_path[pri]);
//                 // cerr << "}" << endl;
//                 // sub躲避，pri减速
//                 if (shelter_vertex[sub].has_value())
//                 {
//                     // 已经在躲避了，停一下
//                     robot_path[pri] = {};
//                     stop_until[pri] = meta.current_flame + 1 * 50;
//                 }
//                 else
//                 {
//                     robot_path[sub] = find_path_square::find_shelter(
//                         sub_path[0], robot_path[pri], meta.robot[sub].goods == 0 ? false : true);
//                     if (robot_path[sub].size() > 2) shelter_vertex[sub] = robot_path[sub].back();
//                 }
//             }
//         }
//     }
// }

void process_anticollision_2(vector<Path> &robot_path)
{
    // 解决单行道死锁
    {
        for (int pri = 1; pri < robot_path.size(); ++pri)
        {
            for (int sub = pri + 1; sub < robot_path.size(); ++sub)
            {
                const auto &pri_path = robot_path[pri];
                const auto &sub_path = robot_path[sub];
                bool need_shelter = false;
                double left_dis_pri = 3.0;
                double left_dis_sub = 3.0;

                for (int i = 0; i < pri_path.size(); ++i)
                {
                    navmesh::Vec2 vec_pri {pri_path[i], pri_path[(i + 1) % pri_path.size()]};
                    navmesh::Segment pri_seg;
                    // 过长截取
                    if (vec_pri.length() >= left_dis_pri)
                        // XXX 万一除0
                        pri_seg = {
                            pri_path[i],
                            {pri_path[i].x + vec_pri.x * left_dis_pri / vec_pri.length(),
                                 pri_path[i].y + vec_pri.y * left_dis_pri / vec_pri.length()}
                        };
                    else
                    {
                        pri_seg = {pri_path[i], pri_path[(i + 1) % pri_path.size()]};
                    }
                    left_dis_pri -= pri_seg.length();

                    for (int j = 0; j < sub_path.size(); ++j)
                    {
                        // 过长截取
                        navmesh::Vec2 vec_sub {sub_path[j], sub_path[(j + 1) % sub_path.size()]};
                        navmesh::Segment sub_seg;
                        if (vec_sub.length() >= left_dis_sub)
                            // XXX 万一除0
                            sub_seg = {
                                sub_path[j],
                                {sub_path[j].x + vec_sub.x * left_dis_sub / vec_sub.length(),
                                     sub_path[j].y + vec_sub.y * left_dis_sub / vec_sub.length()}
                            };
                        else
                        {
                            sub_seg = {sub_path[j], sub_path[(j + 1) % sub_path.size()]};
                        }
                        left_dis_sub -= sub_seg.length();

                        double dis = navmesh::Segment::distance(pri_seg, sub_seg);
                        if (dis <= 1.2) need_shelter = true;

                        if (left_dis_sub <= 1e-6 or need_shelter) break;
                    }
                    if (left_dis_pri <= 1e-6 or need_shelter) break;
                }

                if (need_shelter)
                {
                    if (_USE_LOG_)
                    {
                        cerr << "[info] robot " << pri << " and " << sub << " need shelter " << endl;
                    }
                    robot_path[sub] = find_shelter_path(sub_path,
                        vector<Path>(robot_path.begin(), robot_path.begin() + sub),
                        meta.robot[sub].goods == 0 ? false : true);
                }
            }
        }
    }
}

/*1帧15ms内给出策略*/
void give_pointing()
{
    // 得到path
    vector<Path> robot_path(meta.robot.size());

    // 得到path
    {
        for (int i = 1; i < meta.robot.size(); i++)
        {
            if (areas[i].stations.empty()) continue;
            /* STOP逻辑 */
            if (stop_until[i] != -1)
            {
                if (meta.current_flame < stop_until[i])
                {
                    if (_USE_LOG_)
                    {
                        cerr << "[info][pointing] frame " << meta.current_flame << " robot " << i
                             << "stop until" << stop_until[i] << endl;
                    }
                    continue;
                }
                else
                    stop_until[i] = -1;
            }
            auto &area = areas[i];
            auto &robot = meta.robot[i];

            /* 重新获取任务 */
            if (processing[i] == 0)
            {
                processing[i] = area._give_pointing(i);
                if (processing[i] == 0)
                {
                    if (_USE_LOG_)
                    {
                        cerr << "[waring][pointing] robot " << i << "didn't get route. stop 5s." << endl;
                    }
                    robot_path[i] = {robot.loc, robot.loc};
                    stop_until[i] = meta.current_flame + 2 * 50;
                    if (_USE_LOG_)
                    {
                        cerr << "[info][pointing] [flame=" << meta.current_flame << "] robot " << i
                             << " stop until" << stop_until[i] << endl;
                    }
                    if (meta.current_flame + 900 > ConVar::time_limit)
                        stop_until[i] = ConVar::time_limit;    // 最后时刻完全停止
                    continue;
                }
            }

            auto &route = area.routes[processing[i]];
            /* 处理任务*/
            if (robot.goods == 0)
            {
                if (processing_state[i] == ProcessingState::SELL)    // 3->1
                {
                    if (_USE_LOG_)
                    {
                        cerr << "[info][pointing] [flame=" << meta.current_flame << "] robot " << i
                             << " finished" << area.routes[processing[i]] << endl;
                    }

                    processing[i] = 0;
                    processing_state[i] = ProcessingState::PICKING;
                    continue;
                }
                processing_state[i] = ProcessingState::BUY;    // 1->2

                if (robot.in_station == route.from_station_index)
                {
                    io::instructions.push_back(new io::I_buy(i));
                }
                robot_path[i] = find_path(robot.loc, route.from_station().loc, false);
                // navigate::move_to(
                //     robot, target_station.loc, {meta.station[route.to_station_index].loc}, wait_flame);
                if (robot_path[i].empty())
                {
                    if (_USE_LOG_)
                    {
                        cerr << "[error][__pointing] robot " << i << " 没有得到取from的路径, from"
                             << robot.loc << "-> route" << route << endl;
                    }
                }
            }
            else
            {
                if (processing_state[i] == ProcessingState::BUY)    // 2->3
                    processing_state[i] = ProcessingState::SELL;
                if (robot.in_station == route.target_station_index)
                {
                    io::instructions.push_back(new io::I_sell(i));
                }
                robot_path[i] = find_path(robot.loc, route.target_station().loc, true);
                if (robot_path[i].empty())
                {
                    if (_USE_LOG_)
                    {
                        cerr << "[error][__pointing] robot " << i << " 没有得到至target的path！" << endl;
                    }
                }
            }
        }

        for (int i = 1; i < meta.robot.size(); i++)
            if (processing[i] != 0 and robot_path[i].size() < 2)
            {
                if (_USE_LOG_)
                {
                    cerr << "[error][pointing]: robot " << i << " has no path! with route " << processing[i]
                         << endl;
                }
                robot_path[i] = {meta.robot[i].loc, meta.robot[i].loc};
            }
    }

    // 解决单行道死锁
    {
        process_anticollision_2(robot_path);
    }

    // 调用navigate移动
    {
        for (int i = 1; i < meta.robot.size(); i++)
        {
            auto &robot = meta.robot[i];
            if (robot_path[i].empty()) continue;
            nav_navigate::move_to(robot, robot_path[i]);
        }
    }
}
};


#endif