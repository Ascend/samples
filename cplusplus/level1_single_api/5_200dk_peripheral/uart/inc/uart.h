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

#ifndef UART_ENGINE_H_
#define UART_ENGINE_H_



#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR]  " fmt "\n", ##args)
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
