#ifndef __IOINTERFACE_H__
#define __IOINTERFACE_H__
#include "const.h"
#include "model.h"
#include <iostream>
#include <map>
#include <vector>



namespace io
{
void init(std::istream &io_in)
{

    using model::meta;
    using model::goods;
    using model::workstations;
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
                rob.id = meta.robot.size();
                meta.robot.emplace_back(rob);
            }
            else if (meta.map[x][y] >= '0' && meta.map[x][y] <= '9')
            {
                // 1=0.5m,坐标为中心坐标
                meta.station.emplace_back(meta.map[x][y] - '0', x * 0.5 - 0.25, y * 0.5 - 0.25);
                meta.station.back().id = meta.station.size() - 1;
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
void read_flame(std::istream &io_in)
{
    // loc_to_index init
    int flame, money;
    io_in >> flame >> money;
    model::meta.current_flame = flame;
    model::meta.current_money = money;
    int k;
    io_in >> k;

    for (int i = 0; i < k; ++i)
    {
        Station tmp;
        io_in >> tmp.type >> tmp.loc.x >> tmp.loc.y >> tmp.timeleft >> tmp.material >> tmp.product;
        int index = i+1;
        #ifdef DEBUG
                if (model::meta.station[index].type != tmp.type){
                    std::cerr << "station type error" << std::endl;
                    throw "station type didn't match";
                }
        #endif
        tmp.id = i+1;
        model::meta.station[index] = tmp;
    }
    for (int i = 0; i < ConVar::max_robot; i++)
    {
        Robot tmp;
        io_in >> tmp.in_station >> tmp.goods >> tmp.time_factor >> tmp.crash_factor >> tmp.w >> tmp.v.x
            >> tmp.v.y >> tmp.dirc >> tmp.loc.x >> tmp.loc.y;
        tmp.in_station = tmp.in_station == -1 ? -1 : tmp.in_station + 1;
        int index = i+1;
        tmp.id = i+1;
        model::meta.robot[index] = tmp;
    }
    std::string ok;
    io_in >> ok;
}
/**输出用*/
struct Instruction {
    int robot_id;
    Instruction(int robot_id) : robot_id(robot_id) { }
    virtual void print(std::ostream &io_out) const = 0;
};
struct I_forward : public Instruction {
    double v;    // 设置前进速度
    I_forward(int robot_id, double v) : Instruction(robot_id), v(v) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "forward " << robot_id - 1 << " " << v << std::endl;
    }
};
struct I_rotate : public Instruction {
    double w;    // 设置旋转速度
    I_rotate(int robot_id, double w) : Instruction(robot_id), w(w) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "rotate " << robot_id - 1 << " " << w << std::endl;
    }
};
struct I_buy : public Instruction {
    I_buy(int robot_id) : Instruction(robot_id) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "buy " << robot_id - 1 << std::endl;
    }
};
struct I_sell : public Instruction {
    I_sell(int robot_id) : Instruction(robot_id) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "sell " << robot_id - 1 << std::endl;
    }
};
struct I_destroy : public Instruction {
    I_destroy(int robot_id) : Instruction(robot_id) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "destroy " << robot_id - 1 << std::endl;
    }
};
std::vector<io::Instruction *> instructions; // 输入使用全局变量
/*打印，并清空instructions*/
void print_instructions(std::vector<Instruction *> &instructions, std::ostream &io_out, int flame)
{
    // 打印当前帧数
    io_out << flame << std::endl;
    // 打印并销毁指令
    for (auto &ins : instructions)
    {
        ins->print(io_out);
        delete ins;
    }
    io_out << "OK" << std::endl;
    io_out.flush();
    instructions.clear();
}
}



#endif