#include <fstream>
#include <iostream>
using namespace std;
#include "model.h"


int main()
{
// 打开../Robot/maps/1.txt
#ifdef DEBUG
    ifstream fin("Robot/maps/1.txt");
    if (!fin)
    {
        cerr << "open file error" << endl;
        return 0;
    }
    init(fin);
#else
    init(cin);
#endif
    puts("OK");
    fflush(stdout);


    return 0;
}
