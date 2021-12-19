# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
sys.path.append("../../../../common")
import numpy as np
import cv2
import socket

from acllite_resource import AclLiteResource
from ChessStatusPerception import ChessStatusPerception

MODEL_PATH = "../../model/chess_ckpt_0804_vgg_99.om"


class TcpServer:
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind(("0.0.0.0", 6001))
        self.s.listen(1)

    def recvall(self, sock, count):
        """
        receive info according to the length
        """
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf


def main():
    acl_resource = AclLiteResource()
    acl_resource.init()
    server = TcpServer()
    perception = ChessStatusPerception(MODEL_PATH)
    perception.Init()
    while True:
        print("Waiting for client connection...")
        conn, addr = server.s.accept()
        while True:
            print("Waiting for receive message...")
            length = server.recvall(conn, 16)  # get the length of img file
            if length is None:
                conn.close()
                break
            stringData = server.recvall(conn, int(length))  # according to the length, get the img file
            data = np.frombuffer(stringData, np.uint8)
            decimg = cv2.imdecode(data, cv2.IMREAD_COLOR)  # decode data to image
            print("Received Image Shape: ", decimg.shape)

            chessStatus, chessStatus_real = perception.Process(decimg)
            # ideal chess coordinate
            chessStatusStr = '#'.join([str(item) for item in chessStatus])
            # real chess coordinate
            chessStatus_realStr = '#'.join([str(item) for item in chessStatus_real])
            sendmsg = chessStatusStr + "&" + chessStatus_realStr

            print("Send Message: ", sendmsg)
            conn.sendall(sendmsg.encode())


if __name__ == '__main__':
    main()
