#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"
#include "route_fool.h"

void robot()
{
    io::init(cin);
    route_fool::init();
    puts("OK");
    fflush(stdout);
    /*----------START----------*/
    while (cin.eof() == false)
    {
        cerr << "info: flame read" << endl;
        io::read_flame(cin);
        cerr << "info: flame read end, flame:" << meta.current_flame << endl;
        route_fool::give_pointing();
        io::print_instructions(io::instructions, cout, meta.current_flame);
    }
}
void local()
{
    auto fin = fstream("Robot/maps/1.txt");
    io::init(fin);
    route_fool::init();
    route_fool::give_pointing();
}
int main()
{
    // cerror重定向到文件
    fstream fout("log.txt", ios::out);
    cerr.rdbuf(fout.rdbuf());

    local();
    // robot();
    return 0;
}
