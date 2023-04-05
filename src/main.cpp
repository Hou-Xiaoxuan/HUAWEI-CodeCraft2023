#include "args.h"
#include "find_path_squre.h"
#include "iointerface.h"
#include "model.h"
#include "nav_ear_clipping.h"
#include "nav_model.h"
#include "nav_navigate.h"
#include "route_stupid.h"
#include "trans_map.h"
#include <fstream>
#include <iostream>
#include <optional>
/*clangd的傻逼bug，main.cpp里的第一个函数不能被识别*/
void sb_clangd() { }

void robot()
{
    io::init(std::cin);
    std::cerr << "info: map read end" << std::endl;
    trans_map::solve();
    std::cerr<<"info: trans map end"<<std::endl;
    route_stupid::init();
    std::cerr<<"info: route stupid init end"<<std::endl;
    puts("OK");

    // trans_map(meta.Map::map);

    fflush(stdout);
    /*----------START----------*/
    while (std::cin.eof() == false)
    {
        // cerr << "info: flame read" << endl;
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

void local()
{

    auto fin = std::fstream("./Robot/maps/1.txt");
    if (fin.is_open() == false)
    {
        std::cerr << "[error] map file open failed" << std::endl;
        return;
    }
    io::init(fin);
    std::vector<navmesh::Polygon> polys = trans_map::solve();

    const auto &p = find_path_square::Find_path(meta.robot[1].loc, meta.station[31].loc, false);
    std::vector<navmesh::Vertex> path = p.smooth_path;
    std::cerr << path.size() << std::endl;
    for (auto &i : path)
    {
        std::cerr << i.x << std::endl;
        std::cerr << i.y << std::endl;
    }
}

int main()
{
    // cerror重定向到文件
    auto cerr_buf = std::cerr.rdbuf();
    std::fstream fout("./log.txt", std::ios::out);
    if (fout.is_open())
        std::cerr.rdbuf(fout.rdbuf());
    else
        std::cerr << "[error] log file open failed" << std::endl;
    // local();
    robot();

    fout.close();
    std::cerr.rdbuf(cerr_buf);
    return 0;
}
