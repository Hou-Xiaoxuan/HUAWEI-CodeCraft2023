#ifndef __IOINTERFACE_H__
#define __IOINTERFACE_H__
#include "const.h"
#include "model.h"
#include <iostream>
#include <map>
#include <vector>



namespace io
{
void read_flame(std::istream &io_in)
{
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
        int index = i+1;
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

        int index = i+1;
        meta.robot[index] = tmp;
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