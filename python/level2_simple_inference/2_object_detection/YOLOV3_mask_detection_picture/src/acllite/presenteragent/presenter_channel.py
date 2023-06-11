# !/usr/bin/env python
# -*- coding:utf-8 -*-
import time
import configparser
from multiprocessing import Process, Queue, Manager
import queue
import numpy as np
import sys
# sys.path.append("..")

import acl
import constants as const
from acllite_logger import log_error, log_info
from acllite_image import AclLiteImage


import presenteragent.presenter_datatype as dtype
import presenteragent.presenter_agent as agent
import presenteragent.presenter_message as pm


class PresenterChannel(object):
    """Communication channel between presenter agent and server"""
    def __init__(self, server_ip, port, name='video',
                 content_type=dtype.CONTENT_TYPE_VIDEO):
        """Create instance"""
        self._server_ip = server_ip
        self._port = port
        self._type = content_type
        self._name = name
        self.agent_msg_queue = Queue()
        self.open_status = Manager().Value('i', dtype.STATUS_DISCONNECT)
        self.data_respone_counter = Manager().Value('i', 0)
        self._send_counter = 0

    def startup(self):
        """Create channel and connect with presenter server
        Returns:
            0 connect success
            1 connect failed
        """
        agent_process = Process(
            target=agent.StartPresenterAgent,
            args=(
                self.agent_msg_queue,
                self._server_ip,
                self._port,
                self.open_status,
                self.data_respone_counter))
        agent_process.start()
        time.sleep(0.5)
        self._send_open_channel_request(self._name, self._type)
        return self._wait_open_status(dtype.STATUS_OPENED)

    def _wait_open_status(self, listen_status):
        ret = dtype.STATUS_ERROR
        for i in range(0, 100):
            time.sleep(0.001)
            if self.open_status.value == listen_status:
                log_info("Open status is %d now" % (listen_status))
                ret = dtype.STATUS_OK
                break
        return ret

    def send_message(self, data):
        """Send message to presenter server"""
        self.agent_msg_queue.put(data)
        self._send_counter += 1

    def _send_open_channel_request(self, channel_name, content_type):
        request_msg = pm.open_channel_request(channel_name, content_type)
        self.send_message(request_msg)

    def send_detection_data(self, image_width, image_height,
                            image, detection_result):
        """Send image frame request to presenter server"""
        image_data = None
        if isinstance(image, AclLiteImage):
            image_data = image.byte_data_to_np_array()
        elif isinstance(image, np.ndarray):
            image_data = image
        else:
            log_error("Invalid data to send, ", image)
            return False

        request_msg = pm.image_frame_request(image_width, image_height,
                                             image_data.tobytes(),
                                             detection_result)
        self.send_message(request_msg)

        return True

    def send_image(self, image_width, image_height, image):
        """Send image frame request that only has image to presenter server"""
        detection_result = []
        return self.send_detection_data(image_width, image_height,
                                        image, detection_result)

    def _send_heart_beat_message(self):
        msg = pm.heartbeat_message()
        self.send_message(msg)

    def close(self):
        """Close channel"""
        if self.open_status == dtype.STATUS_EXITTED:
            return

        log_info("Presenter channel close...")
        eos = dtype.FinishMsg("exit")
        self.send_message(eos)
        while self.agent_msg_queue.qsize() > 0:
            time.sleep(0.001)
        self.open_status = dtype.STATUS_EXITTED

    def __del__(self):
        self.close()


def get_channel_config(config_file):
    """Get connect parameters from config file"""
    config = configparser.ConfigParser()
    config.read(config_file)
    presenter_server_ip = config['baseconf']['presenter_server_ip']
    port = int(config['baseconf']['presenter_server_port'])
    channel_name = config['baseconf']['channel_name']
    content_type = int(config['baseconf']['content_type'])

    log_info(
        "presenter server ip %s, port %d, channel name %s, "
        "type %d" %
        (presenter_server_ip, port, channel_name, content_type))
    return presenter_server_ip, port, channel_name, content_type


def open_channel(config_file):
    """Connect with presenter server"""
    server_ip, port, channel_name, content_type = get_channel_config(
        config_file)
    channel = PresenterChannel(server_ip, port, channel_name, content_type)
    ret = channel.startup()
    if ret:
        log_error("ERROR:Open channel failed")
        return None
    return channel
