#include <iostream>
#include <vector>

/*货物*/
struct Goods {
    int number;                // 编号
    int price;                 // 售价
    int cost;                  // 购买价
    std::vector<int> needs;    // 生产所需物品
};

/*工作台*/
struct WorkStation {
    int number;                   // 编号
    int x;                        // x坐标
    int y;                        // y坐标
    int worktime;                 // 生产时间
    std::vector<int> needs;       // 生产所需物品
    std::vector<int> produces;    // 生产物品
};

/*地图*/
struct Map {
    const static int width = 100;
    const static int height = 100;

    std::vector<std::vector<char>> map;
    Map() : map(width + 1, std::vector<char>(height, +1)) { }
};

/*全局变量*/
std::vector<Goods> goods(8);                  // 7种货物
std::vector<WorkStation> workstations(10);    // 9个工作台
Map map;

void init(std::istream& io_in)
{

    /*货物信息*/
    goods[1] = {1, 3000, 6000};
    goods[2] = {2, 4400, 7600};
    goods[3] = {3, 5800, 9200};
    goods[4] = {4, 15400, 22500, {1, 2}};
    goods[5] = {5, 17200, 25000, {1, 3}};
    goods[6] = {6, 19200, 27500, {2, 3}};
    goods[7] = {7, 76000, 105000, {4, 5, 6}};

    /*工作台信息*/
    workstations[1] = {1, 0, 0, 50, {}, {1}};
    workstations[2] = {2, 0, 0, 50, {}, {2}};
    workstations[3] = {3, 0, 0, 50, {}, {3}};
    workstations[4] = {4, 0, 0, 500, {1, 2}, {4}};
    workstations[5] = {5, 0, 0, 500, {1, 3}, {5}};
    workstations[6] = {6, 0, 0, 500, {2, 3}, {6}};
    workstations[7] = {7, 0, 0, 1000, {4, 5, 6}, {7}};
    workstations[8] = {8, 0, 0, 1, {7}, {}};
    workstations[9] = {9, 0, 0, 1, {1, 2, 3, 4, 5, 6, 7}, {}};


    /*读入100*100的地图*/
    for (int i = 1; i <= Map::width; i++)
        for (int j = 1; j <= Map::height; j++)
        {
            io_in >> map.map[i][j];
            if (map.map[i][j] == '.')
                continue;
            else if (map.map[i][j] == 'A')
            {
                // robot，暂时忽略
            }
            else if (map.map[i][j]>='0' &&  map.map[i][j]<='9')
            {
                // 数字1-9
                workstations[map.map[i][j] - '0'].x = i;
                workstations[map.map[i][j] - '0'].y = j;
            } else{
                std::cerr <<"非法输入，map["<<i<<"]["<<j<<"] = "<<map.map[i][j]<<std::endl;
            }
        }
}
