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

