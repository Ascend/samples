# uart

#### 介绍
本样例介绍如何使用Atlas200DK 开发板 UART1 串口发送数据，接收数据

#### 软件架构
软件架构说明
此样例是用MindeStudio 新建一个App工程，在此工程基础上封装了一个 uart类 ，此类在src/uart.cpp文件中, 在main.cpp 中调用uart类函数进行uart发送数据，接收数据。

#### 使用样例前提：让HwHiAiUser用户获取 gpio i2c uart操作权限，登录开发板  编辑 /etc/rc.local 在exit 0 前 增加如下指令。
    echo 504 >/sys/class/gpio/export
    echo 444 >/sys/class/gpio/export
    chown -R HwHiAiUser /sys/class/gpio/gpio444
    chown -R HwHiAiUser /sys/class/gpio/gpio504
    chown -R HwHiAiUser /sys/class/gpio/gpio444/direction
    chown -R HwHiAiUser /sys/class/gpio/gpio504/direction
    chown -R HwHiAiUser /sys/class/gpio/gpio444/value
    chown -R HwHiAiUser /sys/class/gpio/gpio504/value
    chown -R HwHiAiUser /dev/i2c-1
    chown -R HwHiAiUser /dev/i2c-2
    chown -R HwHiAiUser /dev/ttyAMA0
    chown -R HwHiAiUser /dev/ttyAMA1
    usermod -aG HwHiAiUser HwHiAiUser

#### 使用说明

1.  参考 uart类 
2.  参考main.cpp 中 uart_test_axample()函数,包含了打开串口 ctrl_uart.uart_open()，发送数据ctrl_uart.uart_send(....)    ,在while(1)中接收数据ctrl_uart.uart_read(....),并且打印出接收的数据 ctrl_uart.uart_close()关闭打开的串口.
3.  Atlas200DK 串口说明 和对应开发板管脚 UART1是16，18脚，详细查看：
    https://support.huaweicloud.com/productdesc-A200dk_3000/atlas200_DK_pdes_19_0021.html
