/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */

#ifndef ASCENDDK_PRESENTER_AGENT_UTIL_SOCKET_UTILS_H_
#define ASCENDDK_PRESENTER_AGENT_UTIL_SOCKET_UTILS_H_

#include <string>
#include <cstdint>
#include <netinet/in.h>

namespace ascend {
namespace presenter {

namespace socketutils {

// indicating socket error
const int kSocketError = -1;

// indicating socket timeout
const int kSocketTimeout = -11;

/**
 * @brief SetSockAddr
 * @param [in] host_ip              host IP
 * @param [in] port                 port
 * @param [out] addr                address
 * @return true: success, false: failure
 */
bool SetSockAddr(const char *host_ip, uint16_t port, sockaddr_in &addr);

/**
 * @brief set reuse address option
 * @param [in]  socket              file descriptor of the socket
 */
void SetSocketReuseAddr(int socket);

/**
 * @brief set read timeout and write timeout to a socket
 * @param [in]  socket              file descriptor of the socket
 * @param [in]  timeout_in_sec      timeout in second
 */
void SetSocketTimeout(int socket, int timeout_in_sec);

/**
 * @brief Create a new socket
 * @return a file descriptor for the new socket, or SOCKET_ERROR(-1) for errors
 */
int CreateSocket();

/**
 * @brief Open a connection on socket FD to peer at ADDR
 * @param [in] socket               file descriptor of the socket
 * @param [in] addr                 peer address
 * @return 0 on success, -1 for errors.
 */
int Connect(int socket, const sockaddr_in &addr);

/**
 * @brief  Read N bytes into BUF from socket FD.
 * @param [in] socket               file descriptor of the socket
 * @param [out] buffer              buffer to write data to
 * @param [in] size                 size of data to read
 * @return the number read or -1 for errors.
 */
int ReadN(int socket, char *buffer, int size);

/**
 * @brief  Write N bytes into BUF to socket FD.
 * @param [in] socket               file descriptor of the socket
 * @param [in] data                 buffer of data to write to socket
 * @param [in] size                 size of data to write
 * @return the number wrote or -1 for errors.
 */
int WriteN(int socket, const char *data, int size);

/**
 * @brief close the socket
 * @param [in|out]  socket          file descriptor of the socket
 */
void CloseSocket(int &socket);

} /* namespace socketutils */

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_UTIL_SOCKET_UTILS_H_ */
