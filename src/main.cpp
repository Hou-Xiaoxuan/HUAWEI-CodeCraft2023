#include "args.h"
#include "find_path_squre.h"
#include "iointerface.h"
#include "model.h"
#include "nav_model.h"
#include "route_stupid.h"
#include "trans_map.h"
#include <fstream>
#include <iostream>
/*clangd的傻逼bug，main.cpp里的第一个函数不能被识别*/
void sb_clangd() { }

void specific_args()
{

    // 正式赛
    if (meta.station[1].loc.x == 25.25 and meta.station[1].loc.y == 49.25 and meta.station[2].loc.x == 4.25
        and meta.station[2].loc.y == 48.75)
    {
        Args::turn_cost = 5;
        Args::distance_factor = 1;
        int stop_frame_bias = 100;
        double deeper_profit_ratio = 0.5;
        double super_demand_ratio = 0.4;
    }
}
void robot()
{
    io::init(std::cin);
    std::cerr << "[info] map read end" << std::endl;
    trans_map::init();
    std::cerr << "[info] trans map end" << std::endl;
    route_stupid::init();
    std::cerr << "[info] route_stupid init end" << std::endl;
    puts("[info] init OK");

    fflush(stdout);
    /*----------START----------*/
    while (std::cin.eof() == false)
    {
        find_path_square::pre_cnt_path = find_path_square::cnt_path;
        find_path_square::pre_cnt_find_que = find_path_square::cnt_find_que;
        io::read_flame(std::cin);
        std::cerr << "[info] flame read end, flame: " << meta.current_flame << std::endl;
        route_stupid::give_pointing();
        if (_USE_LOG_)
        {
            for (auto in : io::instructions)
            {
                in->print(std::cerr);
            }
        }
        io::print_instructions(io::instructions, std::cout, meta.current_flame);
        if (_USE_LOG_)
        {
            std::cerr << "[info] flame: " << meta.current_flame << " find_path in frame "
                      << find_path_square::cnt_path - find_path_square::pre_cnt_path << std::endl;
            std::cerr << "[info] flame: " << meta.current_flame << " find_que in frame "
                      << find_path_square::cnt_find_que - find_path_square::pre_cnt_find_que << std::endl;
        }
    }
}

void local(const std::string &file)
{
    std::cerr << "chose map " << file << std::endl;
    auto fin = std::fstream(file);
    if (fin.is_open() == false)
    {
        std::cerr << "[error] map file\" " << file << "\" open failed" << std::endl;
        return;
    }
    io::init(fin);
    std::cerr << "[info] map read end" << std::endl;
    trans_map::init();
    std::cerr << "[info] trans map end" << std::endl;
    // route_stupid::init();
    std::cerr << "[info] route_stupid init end" << std::endl;
    puts("[info] init OK");
    // {20.5829, 19.5643}, {23.25, 31.25}
    auto tmp1 = find_path_square::find_path({25.25 + 0.5, 9.75}, meta.station.at(36).loc, false);
    return;
}

int main(int argc, char *argv[])
{
    // cerror重定向到文件
    auto cerr_buf = std::cerr.rdbuf();
    std::fstream fout("./log.txt", std::ios::out);
    if (fout.is_open())
        std::cerr.rdbuf(fout.rdbuf());
    else
        std::cerr << "[error] log file open failed" << std::endl;
    if (argc < 2)
        robot();
    else
    {
        std::string file = argv[1];
        local(file);
    }

    fout.close();
    std::cerr.rdbuf(cerr_buf);
    return 0;
}
