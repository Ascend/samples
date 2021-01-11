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
"""utest channel manager module"""

import os
import sys
import unittest
from unittest.mock import patch
path = os.path.dirname(__file__)
index = path.rfind("ascenddk")
workspace = path[0: index]
path = os.path.join(workspace, "ascenddk/common/presenter/server")
sys.path.append(path)

import common.channel_manager as channel_manager
import common.presenter_socket_server as presenter_socket_server

class TestChannelManager(unittest.TestCase):
    """TestChannelManager"""
    manager = channel_manager.ChannelManager(["image", "video"])

    def test_single_instance(self):
        """test_single_instance"""
        cm_new = channel_manager.ChannelManager(["image", "video"])
        self.assertEqual(cm_new, self.manager)

    def test_create_channel_resource(self):
        """test_create_channel_resource"""
        channel_name = "video"
        channel_fd = 100
        media_type = "video"
        self.manager.register_one_channel(channel_name)
        handler = presenter_socket_server.ChannelHandler(channel_name, media_type)
        self.assertEqual(None, self.manager.get_channel_handler_by_name(channel_name))
        self.assertEqual(None, self.manager.get_channel_handler_by_fd(channel_fd))
        self.manager.create_channel_resource(channel_name, channel_fd, media_type, handler)
        self.assertNotEqual(None, self.manager.get_channel_handler_by_name(channel_name))
        self.assertNotEqual(None, self.manager.get_channel_handler_by_fd(channel_fd))
        self.assertEqual(True, self.manager.is_channel_busy(channel_name))


        self.manager.clean_channel_resource_by_name(channel_name)
        self.assertEqual(None, self.manager.get_channel_handler_by_name(channel_name))
        self.assertEqual(None, self.manager.get_channel_handler_by_fd(channel_fd))
        self.assertEqual(False, self.manager.is_channel_busy(channel_name))

    @patch('common.channel_handler.ChannelHandler')
    def test_close_all_thread(self, mock_class):
        """test_close_all_thread"""
        #prepare
        mock_handler = mock_class.return_value
        mock_handler.close_thread.return_value = None

        channel_name = "video"
        channel_fd = 100
        media_type = "video"
        handler = mock_handler
        self.manager.register_one_channel(channel_name)
        self.assertEqual(None, self.manager.get_channel_handler_by_name(channel_name))
        self.assertEqual(None, self.manager.get_channel_handler_by_fd(channel_fd))
        self.manager.create_channel_resource(channel_name, channel_fd, media_type, handler)
        self.assertNotEqual(None, self.manager.get_channel_handler_by_name(channel_name))
        self.assertNotEqual(None, self.manager.get_channel_handler_by_fd(channel_fd))

        #test
        self.manager.close_all_thread()

        #clean
        self.manager.clean_channel_resource_by_name(channel_name)

    @patch('common.channel_handler.ChannelHandler')
    def test_list_channel_paths(self, mock_class):
        """test_list_channel_paths"""
        #prepare
        mock_handler = mock_class.return_value
        mock_handler.close_thread.return_value = None

        channel_name = "video"
        channel_fd = 100
        media_type = "video"
        handler = mock_handler
        self.manager.register_one_channel(channel_name)
        self.assertEqual(None, self.manager.get_channel_handler_by_name(channel_name))
        self.assertEqual(None, self.manager.get_channel_handler_by_fd(channel_fd))
        self.manager.create_channel_resource(channel_name, channel_fd, media_type, handler)
        self.manager.create_channel_resource(channel_name, channel_fd, media_type, handler)
        self.assertNotEqual(None, self.manager.get_channel_handler_by_name(channel_name))
        self.assertNotEqual(None, self.manager.get_channel_handler_by_fd(channel_fd))

        #test
        self.manager.list_channels()

        #clean
        self.manager.clean_channel_resource_by_name(channel_name)

    def test_register_one_channel_path(self):
        """test_register_one_channel_path"""
        #test
        for i in self.manager.list_channels():
            channel_name = i["name"]
            self.manager.unregister_one_channel(channel_name)

        channel_name = "video1"
        self.assertEqual(False, self.manager.is_channel_exist(channel_name))

        for i in range(1, 10):
            channel_name = "video{}".format(i)
            ret = self.manager.register_one_channel(channel_name)
            self.assertEqual(ret, self.manager.err_code_ok)
            self.assertEqual(True, self.manager.is_channel_exist(channel_name))

        channel_name = "video9"
        ret = self.manager.register_one_channel(channel_name)
        self.assertEqual(ret, self.manager.err_code_repeat_channel)

        channel_name = "video10"
        ret = self.manager.register_one_channel(channel_name)
        self.assertEqual(ret, self.manager.err_code_ok)
        self.assertEqual(True, self.manager.is_channel_exist(channel_name)) 

        channel_name = "video11"
        ret = self.manager.register_one_channel(channel_name)
        self.assertEqual(ret, self.manager.err_code_too_many_channel)

        for i in self.manager.list_channels():
            channel_name = i["name"]
            self.manager.unregister_one_channel(channel_name)

    def test_save_channel_path_image(self):
        """test_save_channel_path_image"""
        channel_name = "image"
        image_data = "image"
        self.manager.register_one_channel(channel_name)

        self.manager.clean_channel_image(channel_name)
        ret = self.manager.get_channel_image(channel_name)
        self.assertEqual(ret, None)

        #test
        self.manager.save_channel_image(channel_name, image_data)

        ret = self.manager.get_channel_image(channel_name)
        self.assertNotEqual(ret, None)

        self.manager.clean_channel_image(channel_name)

        ret = self.manager.get_channel_image(channel_name)
        self.assertEqual(ret, None)

    def test_get_channel_image_null(self):
        """test_get_channel_image_null"""
        channel_name = "abcd"

        ret = self.manager.get_channel_image(channel_name)
        self.assertEqual(ret, None)

if __name__ == '__main__':
    unittest.main()
