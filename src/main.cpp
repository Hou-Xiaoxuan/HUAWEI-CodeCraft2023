#include "args.h"
#include "find_path_squre.h"
#include "iointerface.h"
#include "model.h"
#include "nav_ear_clipping.h"
#include "nav_model.h"
#include "nav_navigate.h"
#include "route_stupid.h"
#include "trans_map.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
/*clangd的傻逼bug，main.cpp里的第一个函数不能被识别*/
void sb_clangd() { }

void robot()
{
    io::init(std::cin);
    std::cerr << "info: map read end" << std::endl;
    trans_map::init();
    std::cerr << "info: trans map end" << std::endl;
    find_path_square::init();
    std::cerr << "info: find path init end" << std::endl;
    route_stupid::init();
    std::cerr << "info: route stupid init end" << std::endl;
    puts("OK");

    // trans_map(meta.Map::map);

    fflush(stdout);
    /*----------START----------*/
    while (std::cin.eof() == false)
    {
        io::read_flame(std::cin);
        std::cerr << "info: flame read end, flame:" << meta.current_flame << std::endl;
        route_stupid::give_pointing();
        for (auto in : io::instructions)
        {
            in->print(std::cerr);
        }
        io::print_instructions(io::instructions, std::cout, meta.current_flame);
    }
}

void local(const std::string& file)
{
    std::cerr << "chose map " << file << std::endl;
    auto fin = std::fstream(file);
    if (fin.is_open() == false)
    {
        std::cerr << "[error] map file\" " << file << "\" open failed" << std::endl;
        return;
    }
    io::init(fin);
    std::cerr << "info: map read end" << std::endl;
    trans_map::init();
    std::cerr << "info: trans map end" << std::endl;
    find_path_square::init();
    std::cerr << "info: find path init end" << std::endl;
    // 计时并输出运行时间


    auto start = std::chrono::steady_clock::now();
    // route_stupid::init();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cerr << "info: route stupid init end, time:" << diff.count() << std::endl;
    std::cerr << "info: route stupid init end" << std::endl;

    // route_stupid::give_pointing();
    auto tmp1 = find_path_square::find_path(meta.robot[1].loc, meta.station[1].loc, false);
    auto tmp2 = find_path_square::find_path(meta.robot[1].loc, meta.station[2].loc, false);
    auto tmp3 = find_path_square::find_path(meta.robot[1].loc, meta.station[3].loc, false);
    auto tmp4 = find_path_square::find_path(meta.robot[1].loc, meta.station[4].loc, false);
    auto tmp5 = find_path_square::find_path(meta.robot[1].loc, meta.station[5].loc, false);
    auto tmp6 = find_path_square::find_path(meta.robot[1].loc, meta.station[6].loc, false);
    auto tmp7 = find_path_square::find_path(meta.robot[1].loc, meta.station[7].loc, false);
    auto tmp8 = find_path_square::find_path(meta.robot[1].loc, meta.station[8].loc, false);
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
