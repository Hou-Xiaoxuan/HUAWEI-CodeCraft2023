#ifndef MODULE_H
#define MODULE_H
#include <iostream>
#include <vector>
struct Point {
    double x;
    double y;
    Point() : x(0), y(0) { }
    Point(double x, double y) : x(x), y(y) { }
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

    Robot() : in_station(-1), goods(0), time_factor(1.0), crash_factor(1.0), loc(0, 0), v(0, 0), w(0), dirc(0) { }
};

/*地图*/
struct Map {
    const static int width = 100;
    const static int height = 100;

    std::vector<std::vector<char>> map;
    std::vector<Station> station;
    std::vector<Robot> robot;
    Map() : map(width + 1, std::vector<char>(height, +1))
    {
        station.reserve(50);
        robot.reserve(5);
    }
};



/*全局变量*/
std::vector<Goods> goods(8);                  // 7种货物
std::vector<WorkStation> workstations(10);    // 9种工作台
Map map;

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
    for (int i = 1; i <= Map::width; i++)
        for (int j = 1; j <= Map::height; j++)
        {
            io_in >> map.map[i][j];
            if (map.map[i][j] == '.')
                continue;
            else if (map.map[i][j] == 'A')
            {
                // robot
                Robot rob;
                rob.loc = Point(i * 0.5 - 0.25, j * 0.5 - 0.25);
                map.robot.emplace_back(rob);
            }
            else if (map.map[i][j] >= '0' && map.map[i][j] <= '9')
            {
                // 1=0.5m,坐标为中心坐标
                map.station.emplace_back(Station {map.map[i][j] - '0', i * 0.5 - 0.25, j * 0.5 - 0.25});
            }
            else
            {
                std::cerr << "非法输入，map[" << i << "][" << j << "] = " << map.map[i][j] << std::endl;
            }
        }
}


// XXX 似乎过度设计了
namespace Ooutput
{
/**输出用*/
struct Instruction {
    int robot_id;
    virtual void print(std::ostream &io_out) const = 0;
};
struct I_forward : public Instruction {
    double v;    // 设置前进速度
    void print(std::ostream &io_out) const override { io_out << "forward " << robot_id << " " << v << std::endl; }
};
struct I_rotate : public Instruction {
    double w;    // 设置旋转速度
    void print(std::ostream &io_out) const override { io_out << "rotate " << robot_id << " " << w << std::endl; }
};
struct I_buy : public Instruction {
    void print(std::ostream &io_out) const override { io_out << "buy " << robot_id << std::endl; }
};
struct I_sell : public Instruction {
    void print(std::ostream &io_out) const override { io_out << "sell " << robot_id << std::endl; }
};
struct I_destroy : public Instruction {
    void print(std::ostream &io_out) const override { io_out << "destroy " << robot_id << std::endl; }
};

void print_instructions(const std::vector<Instruction *> &instructions, std::ostream &io_out)
{
    // 打印并销毁指令
    for (auto &ins : instructions)
    {
        ins->print(io_out);
        delete ins;
    }
    io_out<<"OK"<<std::endl;
    io_out.flush();
}
}
#endif