#ifndef MODULE_H
#define MODULE_H
#include <cmath>
#include <iostream>
#include <vector>
struct Point {
    double x;
    double y;
    Point() : x(0), y(0) { }
    Point(double x, double y) : x(x), y(y) { }

    double static distance(const Point &a, const Point &b)
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    friend ostream &operator<<(ostream &os, const Point &p)
    {
        os << "(" << p.x << "," << p.y << ")";
        return os;
    }
};
/*货物*/
struct Goods {
    int type;                  // 编号
    int price;                 // 售价
    int cost;                  // 购买价
    std::vector<int> needs;    // 生产所需物品
};

/*工作台类型*/
struct WorkStation {
    int type;                     // 编号
    int worktime;                 // 生产时间
    std::vector<int> needs;       // 生产所需物品
    std::vector<int> produces;    // 生产物品
};

/*工作台*/
struct Station {
    int type;
    Point loc;
    int timeleft;    // -1表示空闲,0表示因输出满而停止,>0表示生产剩余时间
    int material;    // 二进制表示物品拥有情况，从低位编码
    int product;     // 1表示有产品，0表示无产品
    Station() : type(-1), loc(-1, -1), timeleft(-1), material(0), product(0) { }
    Station(int type, double x, double y) : type(type), loc(x, y), timeleft(-1), material(0), product(0) { }
};

struct Robot {
    int in_station;    // -1表示不在工作台，否则表示工作台编号
    int goods;         // 0表示无货物，否则表示货物编号
    // 货物系数
    double time_factor;     // 时间系数
    double crash_factor;    // 碰撞系数
    // 位置信息
    Point loc;      // 坐标
    Point v;        // 线速度
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
    }
};



/*全局变量*/
std::vector<Goods> goods(8);                  // 7种货物，从1开始
std::vector<WorkStation> workstations(10);    // 9种工作台，从1开始
Map meta;

void init(std::istream &io_in)
{

    /*货物信息*/
    {
        goods[1] = {1, 3000, 6000};
        goods[2] = {2, 4400, 7600};
        goods[3] = {3, 5800, 9200};
        goods[4] = {
            4, 15400, 22500, {1, 2}
        };
        goods[5] = {
            5, 17200, 25000, {1, 3}
        };
        goods[6] = {
            6, 19200, 27500, {2, 3}
        };
        goods[7] = {
            7, 76000, 105000, {4, 5, 6}
        };
    }
    /*工作台信息*/
    {
        workstations[1] = {1, 50, {}, {1}};
        workstations[2] = {2, 50, {}, {2}};
        workstations[3] = {3, 50, {}, {3}};
        workstations[4] = {
            4, 500, {1, 2},
              {4}
        };
        workstations[5] = {
            5, 500, {1, 3},
              {5}
        };
        workstations[6] = {
            6, 500, {2, 3},
              {6}
        };
        workstations[7] = {
            7, 1000, {4, 5, 6},
              {7}
        };
        workstations[8] = {8, 1, {7}, {}};
        workstations[9] = {
            9, 1, {1, 2, 3, 4, 5, 6, 7},
              {}
        };
    }

    /*读入100*100的地图*/
    for (int y = Map::width; y >= 1; y--)
        for (int x = 1; x <= Map::height; x++)
        {
            io_in >> meta.map[x][y];
            if (meta.map[x][y] == '.')
                continue;
            else if (meta.map[x][y] == 'A')
            {
                // robot
                Robot rob;
                rob.loc = Point(x * 0.5 - 0.25, y * 0.5 - 0.25);
                meta.robot.emplace_back(rob);
            }
            else if (meta.map[x][y] >= '0' && meta.map[x][y] <= '9')
            {
                // 1=0.5m,坐标为中心坐标
                meta.station.emplace_back(meta.map[x][y] - '0', x * 0.5 - 0.25, y * 0.5 - 0.25);
            }
            else
            {
                std::cerr << "非法输入，map[" << y << "][" << x << "] = " << meta.map[x][y] << std::endl;
            }
        }
    // 读入标识结束的“ok”
    std::string ok;
    io_in >> ok;
    std::cerr << "[info] ok = " << ok << std::endl;
}


#endif