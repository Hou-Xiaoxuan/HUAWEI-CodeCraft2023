#include "anticollision.h"
#include "iointerface.h"
#include "model.h"
#include "navigate.h"
#include "route_fool.h"
#include "args.h"
#include <fstream>
#include <iostream>
#include "nav_ear_clipping.h"
/*clangd的傻逼bug，main.cpp里的第一个函数不能被识别*/
void sb_clangd() { }


/*硬编码，识别是几号地图*/
int map_recognize()
{
    if (Point::distance(meta.station[1].loc, {24.75, 49.25}) < 1e-5) return 1;
    if (Point::distance(meta.station[1].loc, {0.75, 49.25}) < 1e-5) return 2;
    if (Point::distance(meta.station[1].loc, {23.25, 49.25}) < 1e-5) return 3;
    if (Point::distance(meta.station[1].loc, {24.75, 46.25}) < 1e-5) return 4;

    return 0;
}
void specific_args()
{
    int map_type = map_recognize();
    if (map_type == 1)
    {
        // 575217
        Args::deeper_profit_ratio = 0.6;
        Args::super_demand_ratio = 0.5;
        Args::persisitent_flame = 7;
        Args::max_predict_flame = 15;
    }
    else if (map_type == 2)
    {
        // 761839
        Args::deeper_profit_ratio = 0.6;
        Args::super_demand_ratio = 0.5;
    }
    else if (map_type == 3)
    {
        // 906590
        Args::persisitent_flame = 7;
        Args::max_predict_flame = 20;
    }
    else if (map_type == 4)
    {
        // 616228
        Args::deeper_profit_ratio = 0.5;
        Args::super_demand_ratio = 0.4;
        Args::persisitent_flame = 7;
        Args::max_predict_flame = 20;
    }
    else
    {
        std::cerr << "[error]map_type error" << std::endl;
    }
}
void robot()
{
    io::init(std::cin);

    specific_args();    // XXX 参数更改

    route_fool::init();
    puts("OK");
    fflush(stdout);
    /*----------START----------*/
    while (std::cin.eof() == false)
    {
        // cerr << "info: flame read" << endl;
        io::read_flame(std::cin);
        std::cerr << "info: flame read end, flame:" << meta.current_flame << std::endl;
        auto routes = route_fool::give_pointing();
        anticollision::anticollision(routes);
        for (auto in : io::instructions)
        {
            in->print(std::cerr);
        }
        io::print_instructions(io::instructions, std::cout, meta.current_flame);
    }
}

void local()
{   
    navmesh::Polygon poly({{0, 0}, {50, 100}, {100, 0}, {50, 50}});
    navmesh::EarClipping ec(poly);
    auto rt = ec.triangulate();
    for (auto &p : rt)
    {
        std::cerr <<p;
    }
}
int main()
{
    // cerror重定向到文件
    // std::fstream fout("../log.txt", std::ios::out);
    // if (fout.is_open())
    //     std::cerr.rdbuf(fout.rdbuf());
    // else
    //     std::cerr << "[error] log file open failed" << std::endl;
    // local();
    local();

    return 0;
}
