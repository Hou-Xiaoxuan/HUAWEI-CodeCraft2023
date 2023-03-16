#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"

int main()
{
    auto file = fstream("my.log");
    cerr.rdbug(file.rdbuf());
    io::init(cin);
    puts("OK");
    fflush(stdout);

    /*----------START----------*/
    int cnt = 0;
    int flag = 1;
    while (cin.eof() == false)
    {
        cerr << "info: flame read" << endl;
        io::read_flame(cin);
        cerr << "info: flame read end, flame:" << meta.current_flame << endl;

        vector<io::Instruction *> instructions;
        cerr << meta.robot[1].w << endl;
        if (cnt % 20 == 0)
        {
            flag = -flag;
        }
        if (flag > 0)
        {
            instructions.push_back(new io::I_rotate(1, M_PI));
        }
        else
        {
            instructions.push_back(new io::I_rotate(1, -M_PI));
        }
        io::print_instructions(instructions, cout, meta.current_flame);
    }
    return 0;
}
