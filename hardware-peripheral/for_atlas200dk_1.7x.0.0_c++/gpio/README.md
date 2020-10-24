# gpio

#### 介绍
Atlas200DK 开发板 用户使用7个GPIO 的使用样例 供用户参考。

#### 软件架构
此样例是用MindeStudio 新建一个App工程，在此工程基础上封装了一个 gpio类 ，此类在src/gpio.cpp文件夹中, main.cpp 中调用gpio类函数进行gpio控制。

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

1.参考 gpio 类

2.参考 main.cpp 

 以下函数返回值为 0->success -1->failed

 （第一个参数是你要操作的GPIO口 id ,数值为 0，1，3，4，5，6，7）分别对应开发板上的7个GPIO口

 gpio io_ctrl;

 io_ctrl.gpio_set_direction(3,1) //设置GPIO3 为输出属性   （第二个参数 为 1 是设置输出属性，0 为输入属性）

 io_ctrl.gpio_set_direction(3,0) //设置GPIO3 为输入属性     

 io_ctrl.gpio_set_value(3,1); // 设置GPIO3 为高电平 ，前提是设置GPIO3为输出属性

 io_ctrl.gpio_set_value(3,0); // 设置GPIO3 为低电平 ，前提是设置GPIO3为输出属性

 io_ctrl.gpio_get_value(3,&value) //读取GPIO0 的电平高低数值放入 value中，0代表低电平  1 代表高电平

 7个IO对应开发板的位置 参考：
https://support.huaweicloud.com/productdesc-A200dk_3000/atlas200_DK_pdes_19_0020.html

 建议：如果想用IO作为输入口循环检测，建议使用GIOP0,1 两个口，读写速度比较快，其他3，4，5，6，7是用扩展芯片扩展的IO口。
