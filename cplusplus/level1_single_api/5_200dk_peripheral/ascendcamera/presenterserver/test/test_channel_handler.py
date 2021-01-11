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
"""utest channel handler module"""

import os
import sys
import time
import unittest
from unittest.mock import patch
path = os.path.dirname(__file__)
index = path.rfind("ascenddk")
workspace = path[0: index]
path = os.path.join(workspace, "ascenddk/common/presenter/server/")
sys.path.append(path)
print(sys.path)
import common.channel_handler as channel_handler

class TestChannelHandler(unittest.TestCase):
    """TestChannelHandler"""

    def test_create_image(self):
        channel_name = "image"
        media_type = "image"
        handler = channel_handler.ChannelHandler(channel_name, media_type)
        self.assertEqual(channel_name, handler.channel_name)
        self.assertEqual(media_type, handler.get_media_type())

    @patch('common.channel_handler.ThreadEvent')
    def test_close_thread(self, mock_class):
        """test_close_thread"""
        thread_event = mock_class.return_value
        thread_event.wait.return_value = None
        thread_event.clear.return_value = None

        channel_name = "video"
        media_type = "video"
        handler = channel_handler.ChannelHandler(channel_name, media_type)
        self.assertEqual(channel_name, handler.channel_name)
        self.assertEqual(handler.close_thread_switch, False)

        handler.close_thread()
        handler.set_heartbeat()
        self.assertEqual(handler.close_thread_switch, True)

    def test_10s_close_thread(self):
        """test_10s_close_thread"""
        channel_name = "video"
        media_type = "video"
        backup = channel_handler.HEARTBEAT_TIMEOUT
        channel_handler.HEARTBEAT_TIMEOUT = 1
        handler = channel_handler.ChannelHandler(channel_name, media_type)
        self.assertEqual(handler.thread.isAlive(), True)
        time.sleep(11)
        self.assertNotEqual(handler.thread.isAlive(), True)
        channel_handler.HEARTBEAT_TIMEOUT = backup

    def test_5s_set_thread_event(self):
        """test_5s_set_thread_event"""
        thread_event = channel_handler.ThreadEvent(1)
        thread_event.wait()
        self.assertEqual(len(thread_event.events), 1)
        thread_event.set()
        time.sleep(6)
        thread_event.set()
        self.assertEqual(len(thread_event.events), 0)


    def test_save_image_image(self):
        """test_save_image_image"""
        channel_name = "image"
        media_type = "image"
        handler = channel_handler.ChannelHandler(channel_name, media_type)

        data = "image data"
        width = 100
        height = 100
        handler.save_image(data, width, height)
        image = handler.get_image()
        self.assertEqual(data, image)

    @patch('common.channel_handler.ThreadEvent')
    def test_save_image_video(self, mock_class):
        """test_save_image_video"""
        thread_event = mock_class.return_value

        channel_name = "video"
        media_type = "video"
        handler = channel_handler.ChannelHandler(channel_name, media_type)
        handler._create_thread()

        data = "image data"
        width = 100
        height = 100
        handler.save_image(data, width, height)
        thread_event.wait.return_value = True
        time.sleep(0.05)
        image, _, _, _ = handler.get_frame()
        #self.assertEqual(data, image)
        time.sleep(2)
        handler.img_data = "image data"
        handler.save_image(data, width, height)

        thread_event.wait.return_value = False
        image, _, _, _ = handler.get_frame()
        self.assertNotEqual(data, image)

        handler.close_thread()
        handler.set_heartbeat()
        self.assertEqual(handler.close_thread_switch, True)


if __name__ == '__main__':
    unittest.main()
