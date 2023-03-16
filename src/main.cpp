#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"
#include "navigate.h"

int main()
{
    auto log = ofstream("../log.txt");
    cerr.rdbuf(log.rdbuf());

    io::init(cin);
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
        // cerr << "info: flame read end, flame:" << meta.current_flame << endl;
        if (meta.robot[robot_id].in_station == targets[target_index])
        {
            target_index++;
        }
        if (target_index == targets.size())
        {
            target_index %= targets.size();
            cerr << "info: over " << cnt++ << endl;
        }
        navigate::move_to(meta.robot[robot_id], meta.station[targets[target_index]].loc);
    }
    return 0;
}
