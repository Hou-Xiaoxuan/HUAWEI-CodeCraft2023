#ifndef __IOINTERFACE_H__
#define __IOINTERFACE_H__
#include "const.h"
#include "model.h"
#include <iostream>
#include <map>
#include <vector>



namespace io
{
std::vector<int> robot_index_to_id;
std::vector<int> robot_id_to_index;
std::vector<int> station_index_to_id;    // index到输入顺序(id)
std::vector<int> station_id_to_index;    // 输入id到index
bool is_init = false;
void read_flame(std::istream &io_in)
{
    if (is_init == false)
    {
        robot_id_to_index.resize(ConVar::max_robot + 1);
        robot_index_to_id.resize(ConVar::max_robot + 1);
        station_id_to_index.resize(ConVar::max_workstation + 1);
        station_index_to_id.resize(ConVar::max_workstation + 1);
    }
    // loc_to_index init
    int flame, money;
    io_in >> flame >> money;
    meta.current_flame = flame;
    meta.current_money = money;
    int k;
    io_in >> k;

    for (int i = 0; i < k; ++i)
    {
        Station tmp;
        io_in >> tmp.type >> tmp.loc.x >> tmp.loc.y >> tmp.timeleft >> tmp.material >> tmp.product;
        if (is_init == false)
        {
            // 找到 station.loc = Pointer(x,y)的下标
            int index = -1;
            for (int j = 1; j < meta.station.size(); ++j)
            {
                if (Point::distance(meta.station[j].loc, tmp.loc) < 1e-3)
                {
                    index = j;
                    break;
                }
            }
            if (index == -1)
            {
                std::cerr << "[error][read_flame] station loc error, loc didn't find:(" << tmp.loc.x << ","
                          << tmp.loc.y << ") with type " << tmp.type << std::endl;
                // 打印所有station的loc
                for (int j = 1; j < meta.station.size(); ++j)
                {
                    std::cerr << "[error][read_flame] staion_index " << j << ":(" << meta.station[j].loc.x
                              << "," << meta.station[j].loc.y << ",type=" << meta.station[j].type
                              << std::endl;
                }
                throw "station loc error";
            }
            station_id_to_index[i] = index;
            station_index_to_id[index] = i;
        }
        int index = station_id_to_index[i];

        #ifdef DEBUG
                if (meta.station[index].type != tmp.type){
                    std::cerr << "station type error" << std::endl;
                    throw "station type didn't match";
                }
        #endif
        meta.station[index] = tmp;
    }
    for (int i = 0; i < ConVar::max_robot; i++)
    {
        Robot tmp;
        io_in >> tmp.in_station >> tmp.goods >> tmp.time_factor >> tmp.crash_factor >> tmp.w >> tmp.v.x
            >> tmp.v.y >> tmp.dirc >> tmp.loc.x >> tmp.loc.y;
        if (is_init == false)
        {
            int index = -1;
            for (int j = 1; j < meta.robot.size(); j++)
            {
                if (Point::distance(meta.robot[j].loc, tmp.loc) < 1e-3)
                {
                    index = j;
                    break;
                }
            }
            if (index == -1)
                throw "robot loc error";
            robot_id_to_index[i] = index;
            robot_index_to_id[index] = i;
        }

        int index = robot_id_to_index[i];
        meta.robot[index] = tmp;
    }
    if (is_init == false)
        is_init = true;

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
        io_out << "forward " << robot_index_to_id[robot_id] << " " << v << std::endl;
    }
};
struct I_rotate : public Instruction {
    double w;    // 设置旋转速度
    I_rotate(int robot_id, double w) : Instruction(robot_id), w(w) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "rotate " << robot_index_to_id[robot_id] << " " << w << std::endl;
    }
};
struct I_buy : public Instruction {
    I_buy(int robot_id) : Instruction(robot_id) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "buy " << robot_index_to_id[robot_id] << std::endl;
    }
};
struct I_sell : public Instruction {
    I_sell(int robot_id) : Instruction(robot_id) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "sell " << robot_index_to_id[robot_id] << std::endl;
    }
};
struct I_destroy : public Instruction {
    I_destroy(int robot_id) : Instruction(robot_id) { }
    void print(std::ostream &io_out) const override
    {
        io_out << "destroy " << robot_index_to_id[robot_id] << std::endl;
    }
};

void print_instructions(const std::vector<Instruction *> &instructions, std::ostream &io_out, int flame)
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
}
}



#endif