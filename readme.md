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
