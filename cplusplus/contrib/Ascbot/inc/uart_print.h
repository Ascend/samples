/**
* @file uart_print.h
*/

#ifndef UART_PRINT_H_
#define UART_PRINT_H_

#define UART_PRINT_ENABLE   1
#include "utils.h"
class uart_print {
public:
    uart_print(void) ;
    ~uart_print(void) ;
    int UPRINT(const char *formt, ...);

private:
    int uart_print_init(void);
    int uart_print_close(void);
    int uart_output_char(void);
    void putccc(char a);
    void putInt(int n);
    void putHex(int n);
    void putfloat(double n);

private:
    int fd ;
    int uart_info_index ;
    char uart_info[1000];
};

#endif // UART_PRINT_H_
