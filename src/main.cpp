#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"
#include "route_fool.h"
int main()
{
    auto fin = fstream("Robot/maps/1.txt");
    io::init(fin);
    route_fool::init();
    route_fool::give_pointing();
    // io::init(cin);
    // route_fool::init();
    // puts("OK");
    // fflush(stdout);
    // /*----------START----------*/
    // while (cin.eof() == false)
    // {
    //     cerr << "info: flame read" << endl;
    //     io::read_flame(cin);
    //     cerr << "info: flame read end, flame:" << meta.current_flame << endl;
    //     route_fool::give_pointing();
    //     io::print_instructions(io::instructions, cout, meta.current_flame);
    // }
    return 0;
}
