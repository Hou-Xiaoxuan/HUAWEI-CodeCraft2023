#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"
#include "navigate.h"
#include "route_fool.h"

void robot()
{
    auto log = ofstream("../log.txt");
    cerr.rdbuf(log.rdbuf());

    io::init(cin);
    route_fool::init();
    puts("OK");
    fflush(stdout);
    /*----------START----------*/
    vector<int> targets = {1, 2, 3};
    int robot_id = 1;
    int target_index = 0;
    int cnt = 0;
    while (cin.eof() == false)
    {
        // cerr << "info: flame read" << endl;
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
