#   =======================================================================
#
# Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   1 Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#   2 Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#   3 Neither the names of the copyright holders nor the names of the
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#   =======================================================================
#
"""utest app manager module"""

import os
import sys
import socket


import time
import unittest
from unittest.mock import patch
path = os.path.dirname(__file__)
index = path.rfind("ascenddk")
workspace = path[0: index]
path = os.path.join(workspace, "ascenddk/common/presenter/server")
sys.path.append(path)

import common.app_manager as app_manager
import common.channel_manager as channel_manager

def create_sock_client():
    """func"""
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    return client_sock

class TestAppManager(unittest.TestCase):
    """TestChannelManager"""
    channel_manager = channel_manager.ChannelManager(["image", "video"])
    manager = app_manager.AppManager()

    def test_single_instance(self):
        """test_single_instance"""
        manager_new = app_manager.AppManager()
        self.assertEqual(manager_new, self.manager)

    @patch("socket.socket.fileno", return_value=1)
    @patch("socket.socket.settimeout", return_value=True)
    def test_register_and_unregister_app(self, mock1, mock2):
        """test_single_instance"""
        app_id = "app1"
        app_id_2 = "app2"
        sock_fd = 1
        sock_fd_2 = 2
        sock = create_sock_client()
        ret = TestAppManager.manager.register_app(app_id, sock)
        self.assertEqual(ret, True)

        ret = TestAppManager.manager.register_app(app_id, sock)
        self.assertEqual(ret, False)

        ret = TestAppManager.manager.get_socket_by_app_id(app_id)
        self.assertEqual(ret, sock)

        ret = TestAppManager.manager.get_socket_by_app_id(app_id_2)
        self.assertEqual(ret, None)

        ret = TestAppManager.manager.get_app_id_by_socket(sock_fd)
        self.assertEqual(ret, app_id)

        ret = TestAppManager.manager.get_app_id_by_socket(sock_fd_2)
        self.assertEqual(ret, None)

        ret = TestAppManager.manager.is_app_exist(app_id)
        self.assertEqual(ret, True)

        ret = TestAppManager.manager.is_app_exist(app_id_2)
        self.assertEqual(ret, False)

        ret = TestAppManager.manager.get_app_num()
        self.assertEqual(ret, 1)

        ret = TestAppManager.manager.list_app()
        self.assertEqual(ret, [app_id])

        TestAppManager.manager.set_heartbeat(sock_fd)
        TestAppManager.manager.unregister_app_by_fd(sock_fd)
        TestAppManager.manager.set_thread_switch()
        time.sleep(1.5)

    @patch("socket.socket.fileno", return_value=1)
    @patch("socket.socket.settimeout", return_value=True)
    def test_app_heartbeat_timeout(self, mock1, mock2):
        """test_single_instance"""
        app_id = "app1"
        sock_fd = 1
        sock = create_sock_client()
        backup = app_manager.HEARTBEAT_TIMEOUT
        app_manager.HEARTBEAT_TIMEOUT = 1
        ret = TestAppManager.manager.register_app(app_id, sock)
        self.assertEqual(ret, True)
        time.sleep(2)
        TestAppManager.manager.unregister_app_by_fd(sock_fd)
        app_manager.HEARTBEAT_TIMEOUT = backup


if __name__ == '__main__':
    unittest.main()
