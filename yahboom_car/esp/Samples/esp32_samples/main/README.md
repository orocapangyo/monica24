# Sample project

可以用以下命令创建一个最简单的esp-idf项目，以main项目为例。

```
idf.py create-project main
```

进入对应的工程目录

```
cd main
```

项目文件列表：

```
├── CMakeLists.txt
└── main
    ├── CMakeLists.txt
    └── main.c
```


设置目标ESP芯片型号: esp32s3

```
idf.py set-target esp32s3
```



打开配置菜单

```
idf.py menuconfig
```



编译固件

```
idf.py build
```



烧录固件并打开串口助手

```
idf.py flash monitor
```





