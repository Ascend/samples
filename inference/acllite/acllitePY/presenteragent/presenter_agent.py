# !/usr/bin/env python
# -*- coding:utf-8 -*-
import time
from threading import Thread
import sys

from acllite_logger import log_error, log_info
from presenteragent.socket_client import AgentSocket
import presenteragent.presenter_message as pm
import presenteragent.presenter_datatype as datatype


class PresenterAgent(object):
    """Message proxy to presenter server"""
    def __init__(self, server_ip, port):
        self.socket = AgentSocket(server_ip, port)
        self._closed = False
        self.heart_beat_thread = None

    def connect_server(self):
        """Connect presenter server"""
        return self.socket.connect()

    def start_heard_beat_thread(self):
        """Start thread that send heardbeat messages"""
        self.heart_beat_thread = Thread(target=self._keep_alive)
        self.heart_beat_thread.start()

    def _keep_alive(self):
        msg = pm.heartbeat_message()

        while True:
            if self._closed:
                log_error("Heard beat thread exit")
                break

            self.socket.send_msg(msg)
            time.sleep(2)

    def exit(self):
        """Proxy exit"""
        self.socket.close()
        self._closed = True


def StartPresenterAgent(
        msg_queue,
        server_ip,
        port,
        open_status,
        data_respone_counter):
    """Startup presenter agent"""
    agent = PresenterAgent(server_ip, port)
    ret = agent.connect_server()
    if ret:
        log_error("Connect server failed, ret =", ret)
        return

    open_status.value = datatype.STATUS_CONNECTED

    while True:
        data = msg_queue.get()
        if data is None:
            continue

        if isinstance(data, datatype.FinishMsg):
            log_info("Receive presenter agent exit notification, queue size ",
                     msg_queue.qsize())
            time.sleep(0.1)
            agent.exit()
            break

        agent.socket.send_msg(data)
        msg_name, msg_body = agent.socket.recv_msg()
        if (msg_name is None) or (msg_body is None):
            log_error("Recv invalid message, message name ", msg_name)
            continue

        if ((open_status.value == datatype.STATUS_CONNECTED) and
                pm.is_open_channel_response(msg_name)):
            log_info("Received open channel respone")
            open_status.value = datatype.STATUS_OPENED
            agent.start_heard_beat_thread()
            log_info(
                "presenter agent change connect_status to ",
                open_status.value)

        if ((open_status.value == datatype.STATUS_OPENED) and
                pm.is_image_frame_response(msg_name)):
            data_respone_counter.value += 1
            #log_info("send ok ", data_respone_counter.value)
