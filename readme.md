#  目录

| 路径  | 解释                              |
| ----- | --------------------------------- |
| src   | c++源代码，上传文件目录           |
| Robot | 判题器&播放器&replay存档&官方示例 |
| build | cmake缓存&可执行文件目录          |
| doc   | 文档                              |



# 运行方式

```
> cd build
> cmake ../src
> make
```



程序不需要读/写文件，使用标准输入输出里交互。

判题器使用：

```
Robot -m <地图> <可执行文件>
```

==这样似乎没办法调试，毕竟Robot没办法暂停==

![image-20230323171518776](/home/xv_rong/.config/Typora/typora-user-images/image-20230323171518776.png)

![image-20230323171544179](/home/xv_rong/.config/Typora/typora-user-images/image-20230323171544179.png)


# 初赛(正式赛)

## 调参纪录 

`map1`:有7，角落分散1 2 3各一个

`map2`:两个7，上下远处个一组4 5 6

`map3`:无7，4 5 6各自扎堆

`map4`:有7，只有一个4，在远处

**有提升的参数与表现**：



```cpp
// defaut
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
```



```cpp
// map2: 730522
Args:: deeper_profit_ratio = 0.5;        // 下一层需求影响，默认0.4


// 745225
Args::deeper_profit_ratio = 0.5;
Args::super_demand_ratio = 0.5;

// 761839
Args::deeper_profit_ratio = 0.6;
Args::super_demand_ratio = 0.5;
```

```cpp
// map1: 543952
ARgs:: deeper_profit_ratio = 0.5;
Args:: persisitent_flame = 5;
Args:: max_predict_flame = 15;

// 550688
Args::deeper_profit_ratio = 0.6;
Args::super_demand_ratio = 0.5;
```







