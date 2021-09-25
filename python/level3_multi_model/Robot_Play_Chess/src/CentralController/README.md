# 中心控制开发文档

中心控制是用来集成各模块并正式实现下棋功能的代码，可以视作本项目的入口函数。

## 1. 总体功能

实现了人机对弈、AI自对弈两个主要功能，同时也支持两个功能之间的相互切换；

机械臂方面默认是双机械臂对弈，一个机械臂代表红方走子，另一个机械臂代表黑方走子；也可以通过修改参数实现单机械臂完成双方的走子操作。

## 2. 概要设计

具体代码参数设计请看源码注释。

### 2.1 结构设计

定义CentralControl类，初始化包括棋盘状态、下棋方、棋局结束标记、各部分socket通信相关变量等；

定义SocketConnet成员函数，实现与各个模块的socket连接，可以自由选择连接的模块；

定义GetChessBoardInfo成员函数，实现通过棋盘理解获得棋盘信息功能；

定义NextMoveViaSocket成员函数，实现通过对弈引擎获得当前局面下，AI下一步的走法；

定义MakeMove成员函数，实现调用机械臂完成给定走法功能；提一下MakeMove_help，在机械臂不使用的时候可以使用这个函数来打印走法，人工完成走子。

定义AI_VS_AI和Human_VS_AI成员函数，实现两种对弈模式。

### 2.2 接口信息

主要分成两部分，一部分是输入的信息，另一部分是输出的信息。

- [Recv from] Webserver的初始化信号：Request Initial Chess Board，长度为27
- [Send to  ] Webserver的棋局信息：#[x, y, c]#[]#[]...#[]，长度512
- [Recv from] Webserver的初始化ACK信号：Initial OK，长度10
- [Recv from] Webserver的对弈模式信号：Human VS AI 或者 AI VS AI，长度为11，若长度不足需要补全到11（Human VS AI长度是11）
- [Send to  ] Webserver的等待红方落子信号：Waiting for the red，长度为19
- [Recv from] Webserver的红方玩家落子信号：#[x_src, y_src, class]#[x_dst, y_dst, class], 长度为32，若长度不足需要补到32
- [Send to  ] Webserver的黑方AI落子信号：#[0, b]#[x, y, c]#[x, y, c]，长度为32，若长度不足用ljust补齐32，0代表是否结束，b代表胜负方

- [Send to  ] 棋盘理解的图片信息：    
- [Recv from] 棋盘理解的棋局信息：[]#[]#...#[]&[]#[]#...#[]，前面是理想坐标，后面是真实坐标。
- [Send to  ] 对弈引擎的棋局信息和下棋方信息： 棋局信息[[x,y,class], [], ...], 下棋方'r' 或者 'b'
- [Recv from] 棋盘理解的落子信息：[6, 2, 6, 4]，长度12
- [Recv from] 棋盘理解的棋局结束信息：0或1，长度1
- [Recv from] 棋盘理解的胜负方信息：r或b，长度1

### 2.3 打印设计

对于上述提到的收发信息的点基本上都设置了打印。如果需要人为确认，可以将代码中的
```py
print("##调试## xxxxxxx")
```
替换为：
```py
input("##调试## xxxxxxx")
```

运行时通过按回车来确认打印数据。


### 2.4 功能函数

包含了若干用来打包发送数据、解析接收数据的函数。

EncodeBoard: 将int[[x, y, c], [],..] 打包成str：[x, y, c]#[]#[]...

EncodeAImv: 将str[x_src, y_src, x_dst, y_dst]、 'r' or 'b'、0 or 1打包成#[is_mate, player]#[x_src, y_src, class_src]#[x_dst, y_dst, class_dst]

DecodeHumanMv: 将#[x, y, c]#[x, y, c]解析成int list：[x_src, y_src, x_dst, y_dst]

IsKillStep: 判断int list：[x_src, y_src, x_dst, y_dst]是否是杀棋走法

MoveStr2List: 将[str: x_src, y_src, x_dst, y_dst] 解析成[int: x_src, y_src, x_dst, y_dst]

功能成员函数:

Ideal2Real: 将下棋指令中的理想坐标[x_idl, y_idl]转化为棋盘格上的真实坐标[x_real, y_real]

ChangeSide: 改变下棋方

ChangeMode: 接收并改变对弈模式

## 3. 运行方式

在运行之前，确认机械臂是否标定完成，确认棋盘理解是否标定完成，

检查端口参数是否配置正确，几个socket通信的地址和端口应当各自对齐，检查机械臂是否按顺序打开（打开顺序决定了机械臂中的USBttyID），

先在200DK上启动棋盘理解脚本，然后在PC上启动cpp的对弈引擎脚本，再启动这个中心控制代码。

之后运行Webserver服务器，按照UI的说明文档进行操作。

## 4. 补充说明

中心控制支持部分模块单独连接来调试对应模块，支持脱离机械臂使用人工走子调试。