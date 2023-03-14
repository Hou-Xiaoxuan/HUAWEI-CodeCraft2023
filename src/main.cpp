#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"

int main()
{
    model::init(cin);
    puts("OK");
    fflush(stdout);

    /*----------START----------*/
    while (cin.eof() == false)
    {
        cerr << "info: flame read" << endl;
        io::read_flame(cin);
        cerr << "info: flame read end, flame:" << meta.current_flame << endl;

        vector<io::Instruction *> instructions;
        instructions.push_back(new io::I_forward(1, 6));
        instructions.push_back(new io::I_forward(2, 6));
        instructions.push_back(new io::I_forward(3, 6));
        instructions.push_back(new io::I_forward(4, 6));
        instructions.push_back(new io::I_rotate(1, -2));
        instructions.push_back(new io::I_rotate(2, -1));
        instructions.push_back(new io::I_rotate(3, 1));
        instructions.push_back(new io::I_rotate(4, 2));
        io::print_instructions(instructions, cout, meta.current_flame);
    }
    return 0;
}
