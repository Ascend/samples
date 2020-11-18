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
"""facial recognition channel handler module"""

import time
import logging
from common.channel_handler import ChannelHandler

HEARTBEAT_TIMEOUT = 100

class FacialRecognitionHandler(ChannelHandler):
    '''FacialRecognitionHandler'''
    def __init__(self, channel_name, media_type):
        '''init func'''
        self.sleep_time = 0.01
        super(FacialRecognitionHandler, self).__init__(channel_name, media_type)

    def save_frame(self, image, face_list):
        """
        Description: save frame info
        Input:
            image: original image data
            face_list: faces info, inlude name, face feature, face coordinate
        Returns: NA
        """
        while self.img_data:
            time.sleep(self.sleep_time)

        # compute fps
        self.time_list.append(self.heartbeat)
        self.image_number += 1
        while self.time_list[0] + 1 < time.time():
            self.time_list.pop(0)
            self.image_number -= 1
            if self.image_number == 0:
                break

        self.fps = len(self.time_list)
        self.img_data = image
        self.face_list = face_list
        self.image_event.set()
        self.set_heartbeat()

    def get_frame(self):
        """
        Description: get frame info
        Input:NA
        Returns:
            {
                "image": self.frame_data,
                "fps": self.fps,
                "face_list": self.face_list
            }
        """
        # wait util receive a frame data, and push it to your browser.
        ret = self.web_event.wait()
        self.web_event.clear()
        # True: _web_event return because set()
        # False: _web_event return because timeout
        if ret:
            return {
                "image": self.frame_data,
                "fps": self.fps,
                "face_list": self.face_list
            }

        return {}

    def frames(self):
        """a generator generates image"""
        while True:
            self.image_event.wait()
            self.image_event.clear()
            if self.img_data:
                yield self.img_data
                self.img_data = None

            # if set _close_thread_switch, return immediately
            if self.close_thread_switch:
                yield None

            # if no frames or heartbeat coming in the last 100 seconds,
            # stop the thread and close socket
            if time.time() - self.heartbeat > HEARTBEAT_TIMEOUT:
                self.set_thread_switch()
                self.img_data = None
                yield None

    def _video_thread(self):
        """background thread to process video"""
        logging.info('create %s...', (self.thread_name))
        for frame in self.frames():
            if frame:
                # send signal to clients
                self.frame_data = frame
                self.web_event.set()

            # exit thread
            if self.close_thread_switch:
                self.channel_manager.clean_channel_resource_by_name(
                    self.channel_name)
                logging.info('Stop thread:%s.', (self.thread_name))
                break
