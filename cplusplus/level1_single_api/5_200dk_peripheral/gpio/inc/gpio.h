/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef GPIO_H_
#define GPIO_H_

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR]  " fmt "\n", ##args)
class gpio {
public:
    gpio(void) ;
    ~gpio(void) ;

    int gpio_set_direction(int pin,int direction);   // pin 0,1 3,4,5,6,7  dir=0 -> input   dir=1 ->ouput
    int gpio_set_value(int pin,int val);       // pin 0,1 3,4,5,6,7  val=0 -> low level   val=1 ->high level
    int gpio_get_value(int pin,int *val);      // pin 0,1 3,4,5,6,7  val=0 -> low level   val=1 ->high level

private:
    int i2c_write(unsigned char slave, unsigned char reg, unsigned char value);
    int i2c_read(unsigned char slave, unsigned char reg, unsigned char *buf);
    int i2c_1_init();
    int PCA6416_gpio_set_direction(int pin,int dir);
    int PCA6416_gpio_set_value(int pin,int val);
    int PCA6416_gpio_get_value(int pin,int *val);
    int ASCEND310_gpio_set_direction(int pin,int dir);
    int ASCEND310_gpio_set_value(int pin,int val);
    int ASCEND310_gpio_get_value(int pin,int *val);

private:
    int fd;
    struct i2c_msg {
        unsigned short addr;     /* slave address */
        unsigned short flags;
        unsigned short len;
        unsigned char *buf;     /* message data pointer */
    };
    struct i2c_rdwr_ioctl_data {
        struct i2c_msg *msgs;   /* i2c_msg[] pointer */
        int nmsgs;              /* i2c_msg Nums */
    };
};

#endif // GPIO_H_
