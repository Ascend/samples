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
"""presenter channel manager module"""

import logging
import threading

# max support 10 channels
MAX_CHANNEL_NUM = 10

# when a channel have receive data,
# the active status will last 3 seconds
ACTIVE_LAST_TIME = 3

class ChannelResource():
    
    def __init__(self, handler, socket=None):
        self.handler = handler
        self.socket = socket

class ChannelFd():
    
    def __init__(self, channel_name, handler):
        self.channel_name = channel_name
        self.handler = handler

class Channel():
    
    def __init__(self, channel_name):
        self.channel_name = channel_name
        self.image = None
        self.rectangle_list = None

class ChannelManager():
    
    __instance = None
    channel_resources = {}
    channel_fds = {}
    channel_list = []
    channel_resource_lock = threading.Lock()
    channel_fds_lock = threading.Lock()
    channel_lock = threading.Lock()
    err_code_ok = 0
    err_code_too_many_channel = 1
    err_code_repeat_channel = 2

    def __init__(self, channel_list=None):
        ''''init'''



    def __new__(cls, channel_list=None):
        
        if cls.__instance is None:
            cls.__instance = object.__new__(cls)
            if channel_list is not None and isinstance(channel_list, list):
                for i in channel_list:
                    cls.channel_list.append(Channel(channel_name=i))
                    logging.info("register channel %s", i)
            
        return cls.__instance

    def _register_channel_fd(self, sock_fileno, channel_name):
        
        if self.channel_fds.get(sock_fileno):
            del self.channel_fds[sock_fileno]
        handler = self.channel_resources[channel_name].handler
        self.channel_fds[sock_fileno] = ChannelFd(channel_name, handler)


    def create_channel_resource(self, channel_name,
                                channel_fd,
                                media_type,
                                handler):
        
        with self.channel_resource_lock:
            log_info = "create channel resource,"
            log_info += " channel_name:%s, channel_fd:%u, media_type:%s"
            logging.info(log_info, channel_name, channel_fd, media_type)
            self.channel_resources[channel_name] = \
                ChannelResource(handler=handler, socket=channel_fd)
            self._register_channel_fd(channel_fd, channel_name)

    def _clean_channel_resource(self, channel_name):

        if self.channel_resources.get(channel_name):
            self.channel_resources[channel_name].handler.close_thread()
            self.channel_resources[channel_name].handler.web_event.set()
            self.channel_resources[channel_name].handler.image_event.set()
            del self.channel_resources[channel_name]
            logging.info("clean channel: %s's resource", channel_name)

    def clean_channel_resource_by_fd(self, sock_fileno):

        with self.channel_fds_lock:
            with self.channel_resource_lock:
                if self.channel_fds.get(sock_fileno):
                    self._clean_channel_resource(
                        self.channel_fds[sock_fileno].channel_name)
                    del self.channel_fds[sock_fileno]

    def clean_channel_resource_by_name(self, channel_name):

        if self.channel_resources.get(channel_name):
            self.clean_channel_resource_by_fd(
                self.channel_resources[channel_name].socket)

    def get_channel_handler_by_fd(self, sock_fileno):

        with self.channel_fds_lock:
            if self.channel_fds.get(sock_fileno):
                return self.channel_fds[sock_fileno].handler
            return None

    def is_channel_busy(self, channel_name):

        with self.channel_resource_lock:
            if self.channel_resources.get(channel_name):
                return True
            return False

    def close_all_thread(self):

        with self.channel_resource_lock:
            for channel_name in self.channel_resources:
                self.channel_resources[channel_name].handler.close_thread()

    def get_channel_handler_by_name(self, channel_name):

        with self.channel_resource_lock:
            if self.channel_resources.get(channel_name):
                return self.channel_resources[channel_name].handler
            return None

    def list_channels(self):

        with self.channel_lock:
            return [{'status': self.is_channel_busy(i.channel_name),
                     'name': i.channel_name} for i in self.channel_list]

    def register_one_channel(self, channel_name):

        with self.channel_lock:
            if len(self.channel_list) >= MAX_CHANNEL_NUM:
                logging.info("register channel: %s fail, \
                             exceed max number 10.", channel_name)
                return self.err_code_too_many_channel
            for i in range(len(self.channel_list)):
                if self.channel_list[i].channel_name == channel_name:
                    logging.info("register channel: %s fail, \
                                 already exist.", channel_name)
                    return self.err_code_repeat_channel

            self.channel_list.append(Channel(channel_name=channel_name))
            logging.info("register channel: %s", channel_name)
            return self.err_code_ok

    def unregister_one_channel(self, channel_name):
        
        with self.channel_lock:
            for i in range(len(self.channel_list)):
                if self.channel_list[i].channel_name == channel_name:
                    self.clean_channel_resource_by_name(channel_name)
                    logging.info("unregister channel: %s", channel_name)
                    del self.channel_list[i]
                    break

    def is_channel_exist(self, channel_name):
        
        with self.channel_lock:
            for i in range(len(self.channel_list)):
                if self.channel_list[i].channel_name == channel_name:
                    return True
            return False

    def save_channel_image(self, channel_name, image_data, rectangle_list):
        
        with self.channel_lock:
            for i in range(len(self.channel_list)):
                if self.channel_list[i].channel_name == channel_name:
                    self.channel_list[i].image = image_data
                    self.channel_list[i].rectangle_list = rectangle_list
                    break

    def get_channel_image(self, channel_name):
        
        with self.channel_lock:
            for i in range(len(self.channel_list)):
                if self.channel_list[i].channel_name == channel_name:
                    return self.channel_list[i].image

            # channel not exist
            return None

    def get_channel_image_with_rectangle(self, channel_name):
        
        with self.channel_lock:
            for i in range(len(self.channel_list)):
                if self.channel_list[i].channel_name == channel_name:
                    return (self.channel_list[i].image, self.channel_list[i].rectangle_list)
            return (None, None)

    def clean_channel_image(self, channel_name):
        
        with self.channel_lock:
            for i in range(len(self.channel_list)):
                if self.channel_list[i].channel_name == channel_name:
                    self.channel_list[i].image = None
                    break
