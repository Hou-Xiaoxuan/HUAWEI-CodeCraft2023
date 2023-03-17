#ifndef MODULE_H
#define MODULE_H
#include <cmath>
#include <iostream>
#include <vector>
// 声明，实现在文件尾部
struct Point {
    double x;
    double y;
    Point() : x(0), y(0) { }
    Point(double x, double y) : x(x), y(y) { }

    double static distance(const Point &a, const Point &b)
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    friend std::ostream &operator<<(std::ostream &os, const Point &p)
    {
        os << "(" << p.x << "," << p.y << ")";
        return os;
    }
};

struct Speed {
    double x;
    double y;
    Speed(double x, double y) : x(0), y(0) { }

    double len() const { return sqrt(x * x + y * y); }

    friend std::ostream &operator<<(std::ostream &os, const Speed &v)
    {
        os << "(" << v.x << "," << v.y << ")";
        return os;
    }
};
/*货物*/
struct Goods {
    int type;                  // 编号
    int cost;                  // 购买价
    int price;                 // 售价
    std::vector<int> needs;    // 生产所需物品
};

/*工作台类型*/
struct WorkStation {
    int type;                  // 编号
    int worktime;              // 生产时间
    std::vector<int> needs;    // 生产所需物品
    int produce;               // 生产物品
};

/*工作台*/
struct Station {
    int id;    // 在数组中的下标
    int type;
    Point loc;
    int timeleft;        // -1表示空闲,0表示因输出满而停止,>0表示生产剩余时间
    int material;        // 二进制表示物品拥有情况，从低位编码
    int with_product;    // 1表示有产品，0表示无产品
    Station() : type(-1), loc(-1, -1), timeleft(-1), material(0), with_product(0) { }
    Station(int type, double x, double y) :
        type(type), loc(x, y), timeleft(-1), material(0), with_product(0)
    { }

    /*方法*/

    // 判断是否有物品
    inline bool goods_exist(int goods_id) const { return (material & (1 << (goods_id - 1))) != 0; }
    inline int product_id() const;
    inline const Goods &product() const;
};

struct Robot {
    int id;            // 在数组中的下标
    int in_station;    // -1表示不在工作台，否则表示工作台编号
    int goods;         // 0表示无货物，否则表示货物编号
    // 货物系数
    double time_factor;     // 时间系数
    double crash_factor;    // 碰撞系数
    // 位置信息
    Point loc;      // 坐标
    Speed v;        // 线速度
    double w;       // 角速度(正：顺时针)，弧度/s
    double dirc;    // 方向,弧度，[-pai,pai]

    Robot() :
        in_station(-1), goods(0), time_factor(1.0), crash_factor(1.0), loc(0, 0), v(0, 0), w(0), dirc(0)
    { }
};

/*地图*/
struct Map {
    const static int width = 100;
    const static int height = 100;

    std::vector<std::vector<char>> map;    // 从1开始
    std::vector<Station> station;          // 从1开始
    std::vector<Robot> robot;              // 从1开始
    int current_flame;
    int current_money;
    Map() : map(width + 1, std::vector<char>(height, +1))
    {
        station.reserve(50);
        robot.reserve(5);
        station.push_back(Station());
        robot.push_back(Robot());
        current_money = ConVar::init_money;
    }
};

namespace model
{
/*全局变量*/
std::vector<Goods> goods(8);                  // 7种货物，从1开始
std::vector<WorkStation> workstations(10);    // 9种工作台，从1开始
Map meta = Map();
}

const Map &meta = model::meta;

int Station::product_id() const { return model::workstations[type].produce; }
const Goods &Station::product() const { return model::goods[product_id()]; }
#endif