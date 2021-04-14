/**
* @file wheel.h
*/

#ifndef WHEEL_H_
#define WHEEL_H_

#include "i2c.h"
#include "utils.h"
class wheel {
public:
    wheel(void) ;
    ~wheel(void) ;

    int wheel_init(void);
    int wheel_left_move(int direction,unsigned char speed);
    int wheel_right_move(int direction,unsigned char speed);
private:
    i2c i2c_ctrl;
};

#endif // WHEEL_H_
