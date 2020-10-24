/**
* @file uart.h
*/

#ifndef UART_ENGINE_H_
#define UART_ENGINE_H_


class uart {
public:
    uart(void) ;
    ~uart(void) ;

    int uart_open(void);
    int uart_close(void);
    int uart_send(char *buffer,int size);
    int uart_read(char *buffer,int size);
    int uart_set_option(int nSpeed, int nBits, char nEvent, int nStop);

private:
    int fd;
};

#endif // UART_ENGINE_H_
