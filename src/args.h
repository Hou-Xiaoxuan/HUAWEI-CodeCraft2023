#ifndef __ARGS__H__
#define __ARGS__H__
namespace Args
{
/*route_fool 参数*/
int _estimated_move_stable_bias = 15;    // 估计的移动稳定误差，默认15
double deeper_profit_ratio = 0.4;        // 下一层需求影响，默认0.4
double super_demand_ratio = 0.35;        // 高层需求影响，默认0.35
int wait_blame = 10;                     // 空等惩罚，默认10
int persisitent_flame = 10;              // 躲避持续时间
int max_predict_flame = 20;              // 躲避预测时间
};
#endif
