#include <fstream>
#include <iostream>
using namespace std;
#include "iointerface.h"
#include "model.h"
#include "test_triangle.h"
int main()
{
    init(cin);
    puts("OK");
    fflush(stdout);

    /*----------START----------*/
    while (cin.eof() == false)
    {
        // cerr << "info: flame read" << endl;
        io::read_flame(cin);
        cerr << "[info]: flame:" << meta.current_flame << endl;
        test_triangle::process_flame();
    }
    return 0;
}
