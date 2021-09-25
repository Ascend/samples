# 象棋对接测试说明
WebServer的目的是展示下棋结果，web server实现与中心控制模块的交互，UI接收web server的信息并完成与人的交互。
中心控制模块启动后webServer才可以连接上。

## 一 配置文件说明

​	在运行前先修改配置文件，配置文件位于 app/config/index.js

```javascript
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.systemConfig = void 0;
const systemConfig = {
    // web服务器启动后的端口号
    port: 8100,
    // 中心控制模块的socket端口号
    socketPort: 8200,
    // 中心控制模块的 域名/ ip
    socketHost: "localhost",
    // 中心控制模块socket传输指令的编码
    encoding: "utf8",
    // 启动用模拟测试
    // 该值为true的时候, 象棋UI界面的控制消息不会通过webserver转发至中心控制模块, ui界面指令将由webserver接管回复,webserver没有判断行为, 下棋步骤将是黑方复制红方行为
    // 该值为false的时候 webserver会将象棋ui界面控制指令转发至中心控制模块
    test: true,
    // 启动时候是否在命令窗口打印日志,  项目目录下有个logs文件夹 该文件夹记录所有webserver消息转日志
    cmdOut: true,
};
exports.systemConfig = systemConfig;

```

## 二 运行说明
1. 服务器开发基于node.js
2. 程序包下自带 离线版 node.exe
3. windows环境下请运行 runStandaloneWindows.bat
4. linux环境下 请运行 runStandalone[根据实际linux系统环境选择].sh   
5. 运行后,在浏览器地址输入服务器地址加 web服务器端口号即可打开象棋ui界面

## 三 访问说明
可执行文件运行起来后，在接入局域网后，直接在浏览器上输入 socketHost:port 即可访问

## 四 接口信息
主要分成两部分，一部分是输入的信息，另一部分是输出的信息。

- [Send to  ] Webserver向中心控制发送初始化信号：Request Initial Chess Board，长度为27
- [Recv from] Webserver接收到发送的棋局信息：#[x, y, c]#[]#[]...#[]，长度512
- [Send to  ] Webserver向中心控制发送的初始化ACK信号：Initial OK，长度10
- [Send to  ] Webserver向中心控制发送的对弈模式信号：Human VS AI 或者 AI VS AI，长度为11，若长度不足需要补全到11（Human VS AI长度是11）
- [Recv from] Webserver接收到等待红方落子信号：Waiting for the red，长度为19
- [Send to  ] Webserver向中心控制发送的红方玩家落子信号：#[x_src, y_src, class]#[x_dst, y_dst, class], 长度为32，若长度不足需要补到32
- [Recv from] Webserver接收到黑方AI落子信号：#[0, b]#[x, y, c]#[x, y, c]，长度为32，若长度不足用ljust补齐32，0代表是否结束，b代表胜负方


## 五 目录说明
因项目依赖环境较大,后续文件更新时,只会更新 webserver 或者 ui界面代码,

如果webserver依赖的node_modules发生变动, 则会附带 node_modules更新包

对应的系统平台 windows  linux等环境运行程序不会更新, 对应的  runStandalone[环境].[bat/sh]文件后续也不会变动

1. webserver代码位于 app/文件夹下,
2. ui 代码在 public文件夹下

## 五 文档说明
具体的设计文档说明见Resource目录下的  基于CANN的中国象棋对弈机器人UI模块业务及接口介绍.pdf 文件

