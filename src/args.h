#ifndef __ARGS__H__
#define __ARGS__H__
#define _USE_LOG_ true
namespace Args
{
/*route_fool 参数*/
int _estimated_move_stable_bias = 15;    // 估计的移动稳定误差，默认15
int turn_cost = 5;                       // 转弯消耗，默认10
double distance_factor = 1;              // 距离倍数，默认5
int stop_frame_bias = 100;               // 结束帧数，默认500
double deeper_profit_ratio = 0.5;        // 下一层需求影响，默认0.4
double super_demand_ratio = 0.4;         // 高层需求影响，默认0.35
int wait_blame = 100;                    // 空等惩罚，默认10


int persisitent_flame = 10;    // 躲避持续时间
int max_predict_flame = 20;    // 躲避预测时间
};
#endif
