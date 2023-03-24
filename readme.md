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



# 初赛(正式赛)

## 调参纪录 

`map1`:有7，角落分散1 2 3各一个

`map2`:两个7，上下远处个一组4 5 6

`map3`:无7，4 5 6各自扎堆

`map4`:有7，只有一个4，在远处

**route_fool**参数：

| material | wai t_flame | demand | map1                            | map2   | map3   | map4   | 总分                             | 其他逻辑 |
| -------- | ----------- | ------ | ------------------------------- | ------ | ------ | ------ | -------------------------------- | -------- |
| 0.4      | 10          | 0.35   | 509367                          | 741760 | 872447 | 578181 | <font color='red'>2701755</font> |          |
| 0.5      | 10          | 0.35   | <font color='red'>551353</font> | 727208 | 857851 | 571735 | 2708147                          |          |





