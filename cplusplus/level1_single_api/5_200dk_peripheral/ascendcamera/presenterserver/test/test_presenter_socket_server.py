"""utest presenter socket server module"""

# -*- coding: UTF-8 -*-
#
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
import os
import sys
import time
import unittest
import select
import socket
from unittest.mock import patch
import struct

path = os.path.dirname(__file__)
index = path.rfind("ascenddk")
workspace = path[0: index]
path = os.path.join(workspace, "ascenddk/common/presenter/server")
sys.path.append(path)

import common.channel_manager as channel_manager
import common.channel_handler as channel_handler
import common.presenter_message_pb2 as pb
from face_detection.src.face_detection_server import FaceDetectionServer


IMAGE_DATA = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xfe\x00\x13Created with GIMP\xff\xdb\x00C\x00\x03\x02\x02\x03\x02\x02\x03\x03\x03\x03\x04\x03\x03\x04\x05\x08\x05\x05\x04\x04\x05\n\x07\x07\x06\x08\x0c\n\x0c\x0c\x0b\n\x0b\x0b\r\x0e\x12\x10\r\x0e\x11\x0e\x0b\x0b\x10\x16\x10\x11\x13\x14\x15\x15\x15\x0c\x0f\x17\x18\x16\x14\x18\x12\x14\x15\x14\xff\xdb\x00C\x01\x03\x04\x04\x05\x04\x05\t\x05\x05\t\x14\r\x0b\r\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\xff\xc2\x00\x11\x08\x00\x80\x00\x80\x03\x01\x11\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x1c\x00\x01\x01\x00\x02\x03\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x03\x07\x04\x06\x08\x05\x02\xff\xc4\x00\x15\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\xff\xda\x00\x0c\x03\x01\x00\x02\x10\x03\x10\x00\x00\x01\xfc\x80P\x00(\x00\xa0\xa0\xccPXP\x02\x80\n\x003\x94\x00a>\x01\xf0Nq\xdd\xc0(\x00\xe4\x00Phs\xaa\x98\x0f\xa2zd\x14\x02\x838\x05\x00\x1aP\xe9G\xa8@(J\xa31A@\x06\x8f:9\xea\x90P\x009\x00\x14\x00h\x93\xa2\x1e\xb2(\x00\xa0\xce\n\x00\x06\x84:\t\xeb\xb0\x01@3\x82\x80\x01\xe7\xe3_\x9e\xc2\x05\tT\x0c\xe0\xa0\x02\x9ex5\xe1\xec\xb0\x01@3\x82\x82\x9a\xf4\xd2\x07K>)\xb3\xcd\x84n\xf0P\x0c\xe0\xa0\x1d\\\xd7\x80\x03\xb4\x9b\x10\x14\x038(\x00\xa0\xa0\x02\x80\x0e@\x05\x00\x02\x80\n\n\x0c\xc5\x05\x00\x02\x80\n\x003\x94\x00\n\n\x00\x05\x00\xff\xc4\x00\x1f\x10\x01\x01\x00\x02\x01\x05\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x12\x05\x06\x04\x01\x03\x07\x116\x17\x02\xff\xda\x00\x08\x01\x01\x00\x01\x05\x02\xf4\xf4\xf4\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xbb\xbd\xce\xdfc\xa7\x7fc\xc6q\xdf\xde\xf1\x8e\xfe:\xe3\xb6\xce\x1eO\x99)JR\x94\xa5)J[\xd7\x1f\xb9\xde\xcdv5|\xa7%\x96\xc1rp\xbf\xce\x9d\xf4r\x94\xa5)JR\x94\xa5)y#\xa7\xa6\x99\xf4\xb2\x94\xa5)JR\x94\xa5)y3\xa7\xa6\x93\xf4\xf2\x94\xa5)JR\x94\xa5)yC\xa7\xa6\x8d\xf52\x94\xa5)JR\x94\xa5)yS\xa7\xa6\x89\xf5r\x94\xa5)JR\x94\xa5)yc\xa7\xa6\x83\xf5\xb2\x94\xa5)JR\x94\xa5)yo\xa7\xa7\x8f\xbe\xbaR\x94\xa5)JR\x94\xb7|\xff\x00#Z\xc5~\xaf\x96l{_/ga\xb2\xbd\xdc\x1eK\xf5|\xb3H\xddy\xdb.VR\x94\xa5)JR\xd8\xb5\xbe>\xcb\xc2\xfc\x8f\x10\xfc\x8f\x10\xfc\x8b\x10\xfc\x8b\x10\xd7tN\x0e\xb3\xcd\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xa5)JR\x94\xbd==?\xff\xc4\x00\x14\x11\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\xff\xda\x00\x08\x01\x03\x01\x01?\x01\x00\x7f\xff\xc4\x00\x14\x11\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\xff\xda\x00\x08\x01\x02\x01\x01?\x01\x00\x7f\xff\xc4\x004\x10\x00\x00\x05\x01\x01\x0e\x04\x07\x01\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x11\x000\x05\x13"14AQa\x82\x83\x92\xb2\xc2\xd2\x12!r\xb1\x14#BP`\xb3\xd1\xc1\xff\xda\x00\x08\x01\x01\x00\x06?\x02\xfb,\xa8r\xa6\x1aL1XO\x13\x1fF\x17\xb5@\x15s\xeb)C\xfbI\xb6I5\x80\xe7\x98\x13\x80F)\xd3j\x88&\x99\x94\x1b\xc0y\x14\'\xea5`\xb3P\xbe\xbc\x1fzD\\\xf8>l\xc0\x14g\x14\x7fi\xa6\xd7 \xdb\xdc\xed\xe7M3\xdb\xe4\x1b{\x9b\xbc\xe9\xa6{|\x83osw\x9d4\xcbo\x90\xd6\xf73{\xd3Lv\xf9\ror\xf7\xbd\x14\xc7o\x90\xd6\xf7+{\xd1L6\xff\x00Y\xadRr\xd8\x89\x1c\xe6X\x13\x85@D"\x04s\x0e\xaa\xc9\xd9p\x1b\xba\x9b\xfcRh\xa7x\xf1xo !\x8e4\x88\xe8\xa4^\xa0R\x19T\xa6\x01L^a\x1f\xedd\xec\xb8\r\xddJ\xb6r\x93r\x10\xa8\x8a\x92\x91D\x06d\x038\xeb\xb3#g\'T\x84*\x97\xc9H@\x06`C8k\xac\xa1\xef\x19;k({\xc6N\xda\xca\x1e\xf1\x93\xb6\xb2\x87\xbcd\xed\xa3\xbal\xab\x83\x9c\xc9\xde\xe1S\x00\x84H\x0e`\xd5\xf87\xff\xc4\x00&\x10\x00\x02\x01\x03\x01\x07\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x11!\x00@A1\x10 0Qa\xa1\xc1P`q\x81\x91\xff\xda\x00\x08\x01\x01\x00\x01?!\xf4^\x00\xe2\x11\x06tM\x8c;\xd36<\x9f\xcdJ\xad\xe8]\xc2\xa2k\xe1\xc0\xb0.\\S\x1f\xd5\xedv_E*Hr\xbe*^\x83\xaba\xa8\xff\x00\x14\x0c6\x17\xf9\x08E\xd8\x04\x00\x08\xc1\xd8$\xc7\xf1\xc1\x82\xc1\x7f (\xbb\x03\x04" ;\x07\x0c\xff\x0001qg\xf0\x1b\xf3\xd6\x83\x18\x98lo\xf2\xa2\r\xac\xfa\rC\x118\x93i\xb4A\xd0\xb3\xb3\xb8\xdf\xbdJ3\x98\x97\x0c\x02\x06g\nA\x90D\xb7n\xfdz\xf4\x90\xfcmF\x031\xf627\xff\x00\xc4R\x00C\x10\x01\xff\x00\xff\xda\x00\x0c\x03\x01\x00\x02\x00\x03\x00\x00\x00\x10\x92\t$\x12A\x00\x82 \x04\x12A$\x82H\x00\x00A$\x02\x01$\x92\x01\x00\x02\t$\x12A@\x82\x00 \x12\x01$\x12\x01$\x90\t\x04\x90I \x80\x08$\x10I$\x12\n\x00\x10I\x00\x00\x08$\x90@$\x82H$\x90H\x00\x90\x08$\x90I\x04\x00A$\x02\t$\x12@\x00\x82\x00\x04\x12A$\x82I\x04\x00\x08\x00\xff\xc4\x00\x14\x11\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\xff\xda\x00\x08\x01\x03\x01\x01?\x10\x00\x7f\xff\xc4\x00\x14\x11\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\xff\xda\x00\x08\x01\x02\x01\x01?\x10\x00\x7f\xff\xc4\x00"\x10\x00\x03\x00\x02\x02\x02\x03\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x01q\x11a!1\x10QA\x81\xa1\x91 \xd1\xff\xda\x00\x08\x01\x01\x00\x01?\x10\xcc\xcc\xccLe\xe4\xcf\xc6I$\x82\x08$\x92E\xa1$\x92H\xb4$\x92H\x16\xa2\xd4\x92I Z\x12I\xf2\xc6\x7f@dq\x89|\x99?\x94\x1e\xfd\x0bM\xfd\x9f\x87\x17\xae\t\xcce\xb5\xd3\xf4\x9f8$\x92\x08$\x92| Z\x10I\xf9O\xc3B38\xf8\xc1\xc2\xa3\xb7\xe1\xdb\x03\x97FR_\x06\x1b\xe9\x9bp\x98z0\xf4@\xb4\x16\x84\x10I"\xe5\xd1$\x92I\xf7\x85\x8b{\x0c\x92H\xb5$\x92I\x16\xa4\x92I"\xd4\xfb@\xc5\xfd\x9eI\x16\x82\xd0\x92I$\x81h,\xbc\xf0}\xf9\xed\xe2I$\x10@\xb40\xf4I>\x12I$\x9fz8\xa7\xb0I$\x8bR\t$\x91jA$\x92-\x0f\xb4,S\xd9\xa4\x91j,>\x08$\x93/D\x0bO$\x90}\xf0l\xef7\x82\x05\xa1>2H\xb5\x16\xa4\x89*zK3I\x9e\\\xd9\xc6\x1b\xe3\xd0k\xfe\xd7\n]c\x83\x18\xed\xe7<c\xe1\x87\xc3\xe2\x1fr\x16\x13\x94\xbb\xeb\xc1\xa7\xd4\xbd5\x196\x8f\x0e,g)s\xeeI\x16\xa4\x92H\x8f^\x8e\x1f\x04\x8d\xfe\xea\xca\x12m\x1e\x1cX\xceR\xe7\xdf\xf8\xe1`\xc1\x84Zuu\x89\xa4\xdf.L\xe3\r\xf1\xeaE\xa1$\x92-\x05\xa1$\x10e\xe8\x81hA$\x90-\x08\xf2\xc9"\xd4\x82I$\x82H$\x91h#$\x92I\xdf\xa2I$\x92\x05\xa9$\x92v\xe8\x82I\'\xc1jI$\x9cL|\x98\x18\x18\x18\x98\x98\x9f\xff\xd9'


PB_OPEN_CHANNEL_REQUEST_FULL_NAME = pb._OPENCHANNELREQUEST.full_name
PB_HEART_BEAT_MESSAGE_FULL_NAME = pb._HEARTBEATMESSAGE.full_name
PB_PRESENT_IMAGE_REQUEST_FULL_NAME = pb._PRESENTIMAGEREQUEST.full_name


PB_OPEN_CHANNEL_ERROR_NONE = pb.kOpenChannelErrorNone
PB_OPEN_CHANNEL_ERROR_NO_SUCH_CHANNEL = pb.kOpenChannelErrorNoSuchChannel
PB_OPEN_CHANNEL_ERROR_CHANNEL_ALREADY_OPENED = pb.kOpenChannelErrorChannelAlreadyOpened
PB_OPEN_CHANNEL_ERROR_OTHER = pb.kOpenChannelErrorOther
PB_CHANNEL_CONTENT_TYPE_IMAGE = pb.kChannelContentTypeImage
PB_CHANNEL_CONTENT_TYPE_VIDEO = pb.kChannelContentTypeVideo
PB_IMAGE_FORMAT_JPEG = pb.kImageFormatJpeg
PB_PRESENT_DATA_ERROR_NONE = pb.kPresentDataErrorNone
PB_PRESENT_DATA_ERROR_UNSUPPORTED_TYPE = pb.kPresentDataErrorUnsupportedType
PB_PRESENT_DATA_ERROR_UNSUPPORTED_FORMAT = pb.kPresentDataErrorUnsupportedFormat
PB_PRESENT_DATA_ERROR_OTHER = pb.kPresentDataErrorOther

CHANNEL_MANAGER = channel_manager.ChannelManager(["image", "video"])
HOST = "127.127.0.1"
PORT_BEGIN = 20000

def get_socket_server_addr():
    """func"""
    global PORT_BEGIN
    server_addr = (HOST, PORT_BEGIN)
    for i in range(PORT_BEGIN, 65535):
        ret = os.system("netstat  -nl | grep {}".format(PORT_BEGIN))
        if ret != 0:
            server_addr = (HOST, PORT_BEGIN)
            break
        else:
            print("port conflict:%u", PORT_BEGIN)
            PORT_BEGIN += 1
    PORT_BEGIN += 1
    return server_addr

def create_sock_client(server_address):
    """func"""
    try:
        client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_sock.connect(server_address)
        client_sock.setblocking(1)

        return client_sock
    except socket.error:
        return None

def protobuf_open_channel(channel_name, media_type):
    """func"""
    open_channel_request = pb.OpenChannelRequest()
    open_channel_request.channel_name = channel_name
    open_channel_request.content_type = media_type
    return open_channel_request.SerializeToString()

def protobuf_image_request(image_format):
    """func"""
    send_request = pb.PresentImageRequest()
    send_request.format = image_format
    send_request.width = 128
    send_request.height = 128
    send_request.data = IMAGE_DATA

    return send_request.SerializeToString()

def protobuf_heartbeat_request():
    """func"""
    heartbeat_request = pb.HeartbeatMessage()

    return heartbeat_request.SerializeToString()

def open_channel(client, channel_name, media_type):
    """func"""
    try:
        message_name = pb._OPENCHANNELREQUEST.full_name
        message_name_size = len(pb._OPENCHANNELREQUEST.full_name)
        image_data = protobuf_open_channel(channel_name, media_type)
        image_data_size = len(image_data)
        message_total_size = 5 + message_name_size + image_data_size
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-8") + image_data
        client.sendall(message_data)
        return True
    except socket.error:
        return False

def open_channel_with_size_error(client, channel_name, media_type):
    """func"""
    try:
        message_name = pb._OPENCHANNELREQUEST.full_name
        message_name_size = len(pb._OPENCHANNELREQUEST.full_name)
        image_data = protobuf_open_channel(channel_name, media_type)
        image_data_size = len(image_data)
        #message_total_size = 5 + message_name_size + image_data_size
        message_total_size = 5 + message_name_size - 1
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-8") + image_data
        client.sendall(message_data)
        return True
    except socket.error:
        return False
def open_channel_with_protobuf_error(client):
    """func"""
    try:
        message_name = pb._OPENCHANNELREQUEST.full_name
        message_name_size = len(pb._OPENCHANNELREQUEST.full_name)
        image_data = bytes("virus evil", encoding="utf-8")

        image_data_size = len(image_data)
        message_total_size = 5 + message_name_size + image_data_size
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-8") + image_data
        client.sendall(message_data)
        return True
    except socket.error:
        return False

def open_channel_with_encode_message_name_exception(client, channel_name, media_type):
    """func"""
    try:
        message_name = pb._OPENCHANNELREQUEST.full_name
        message_name_size = len(pb._OPENCHANNELREQUEST.full_name)
        image_data = protobuf_open_channel(channel_name, media_type)
        image_data_size = len(image_data)
        message_total_size = 5 + message_name_size + image_data_size
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-32") + image_data
        client.sendall(message_data)
        return True
    except socket.error:
        return False
def send_one_image_message(client, image_format):
    """func"""
    if image_format is None:
        image_format = PB_IMAGE_FORMAT_JPEG
    message_name = pb._PRESENTIMAGEREQUEST.full_name
    message_name_size = len(pb._PRESENTIMAGEREQUEST.full_name)
    image_data = protobuf_image_request(image_format)

    image_data_size = len(image_data)
    message_total_size = 5 + message_name_size + image_data_size
    message_head = (socket.htonl(message_total_size), message_name_size)
    s = struct.Struct('IB')
    packed_data = s.pack(*message_head)
    message_data = packed_data + bytes(message_name, encoding="utf-8") + image_data

    client.sendall(message_data)

    return True


def send_one_image_message_two_step(client, image_format):
    """func"""
    try:
        if image_format is None:
            image_format = PB_IMAGE_FORMAT_JPEG
        message_name = pb._PRESENTIMAGEREQUEST.full_name
        message_name_size = len(pb._PRESENTIMAGEREQUEST.full_name)
        image_data = protobuf_image_request(image_format)

        image_data_size = len(image_data)
        message_total_size = 5 + message_name_size + image_data_size
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-8") + image_data
        client.sendall(message_data[0:len(message_data)-10])
        time.sleep(0.1)
        client.sendall(message_data[len(message_data)-10:len(message_data)])

        return True
    except socket.error:
        return False

def send_one_image_message_protobuf_format_err(client):
    """func"""
    try:

        message_name = pb._PRESENTIMAGEREQUEST.full_name
        message_name_size = len(pb._PRESENTIMAGEREQUEST.full_name)
        image_data = bytes("virus evil", encoding="utf-8")

        image_data_size = len(image_data)
        message_total_size = 5 + message_name_size + image_data_size
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-8") + image_data
        client.sendall(message_data)
        return True
    except socket.error:
        return False

def send_one_heartbeat_message(client):
    """func"""
    try:
        message_name = pb._HEARTBEATMESSAGE.full_name
        message_name_size = len(pb._HEARTBEATMESSAGE.full_name)
        heartbeat_data = protobuf_heartbeat_request()

        heartbeat_data_size = len(heartbeat_data)
        message_total_size = 5 + message_name_size + heartbeat_data_size
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-8") + heartbeat_data
        client.sendall(message_data)
        return True
    except socket.error:
        return False

def send_one_not_recognize_message(client):
    """func"""
    try:
        message_name = "abc"
        message_name_size = len(message_name)
        message_data = "virus"

        message_data_size = len(message_data)
        message_total_size = 5 + message_name_size + message_data_size
        message_head = (socket.htonl(message_total_size), message_name_size)
        s = struct.Struct('IB')
        packed_data = s.pack(*message_head)
        message_data = packed_data + bytes(message_name, encoding="utf-8") \
                       + bytes(message_data, encoding="utf-8")
        client.sendall(message_data)
        return True
    except socket.error:
        return False

def response_open_channel(client):
    """func"""
    try:
        message_head = client.recv(5)
        s = struct.Struct('IB')
        (message_len, messagename_len) = s.unpack(message_head)
        message_len = socket.ntohl(message_len)
        client.recv(messagename_len)
        message_body = client.recv(message_len -5 -messagename_len)
        open_channel_response = pb.OpenChannelResponse()
        open_channel_response.ParseFromString(message_body)
        if open_channel_response.error_code == PB_OPEN_CHANNEL_ERROR_NONE:
            return True
        return open_channel_response.error_code

    except socket.error:
        return False

def response_image_request(client):
    """func"""
    try:
        message_head = client.recv(5)
        s = struct.Struct('IB')
        (message_len, messagename_len) = s.unpack(message_head)
        message_len = socket.ntohl(message_len)
        client.recv(messagename_len)
        message_body = client.recv(message_len -5 -messagename_len)
        image_request_response = pb.PresentImageResponse()
        image_request_response.ParseFromString(message_body)
        if image_request_response.error_code == PB_PRESENT_DATA_ERROR_NONE:
            return True
        return image_request_response.error_code

    except socket.error:
        return False

def socket_error(fd, conns, messages):
    """func"""
    raise socket.error

def mock_recv(len):
    raise socket.error


class Test_FaceDetectionServer(unittest.TestCase):
    """Test_FaceDetectionServer"""

    def test_create_presenter_agent_server(self):
        """utest"""
        ret = os.system("netstat  -nl | grep 10086")
        if ret != 0:
            port = 10086
        else:
            port = 30086

        server_address = ("127.0.0.1", port)
        server = FaceDetectionServer(server_address)
        port = server_address[1]
        if port == 10086:
            ret = os.system("netstat  -nap | grep 10086")
        else:
            ret = os.system("netstat  -nap | grep 30086")

        self.assertEqual(ret, 0)
        server.stop_thread()

    def test_accept_socket(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        server.stop_thread()

    @patch('select.epoll')
    def test_accept_sockt_error(self, mock_epoll):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)
        epoll = mock_epoll.return_value
        epoll.poll.side_effect = [[[server._sock_server.fileno(), select.EPOLLIN]]]
        time.sleep(0.1)

        server.stop_thread()

    def test_receive_video_flow(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        #test video
        channel_name = "video"
        media_type = PB_CHANNEL_CONTENT_TYPE_VIDEO

        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)

        ret = response_open_channel(client)
        self.assertEqual(ret, True)

        ret = send_one_image_message(client, None)
        self.assertEqual(ret, True)

        #clean
        client.close()
        time.sleep(0.1)
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()


    def test_receive_image(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        #test image
        channel_name = "image"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, True)

        ret = send_one_image_message(client, None)
        self.assertEqual(ret, True)

        #clean
        client.close()
        time.sleep(0.1)
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()

    def test_receive_image_by_2_step(self):
        """utest"""
        server_address = get_socket_server_addr()
        print(server_address)
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        #test image
        channel_name = "image_2_step"
        CHANNEL_MANAGER.register_one_channel(channel_name)
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, True)

        ret = send_one_image_message_two_step(client, None)
        self.assertEqual(ret, True)

        #clean
        client.close()
        time.sleep(0.1)
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        CHANNEL_MANAGER.unregister_one_channel(channel_name)
        server.stop_thread()

    def test_open_channel_fail(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        #not exist channel name
        channel_name = "non_exist"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, PB_OPEN_CHANNEL_ERROR_NO_SUCH_CHANNEL)

        #clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()

    def test_open_channel_fail_for_media_type(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        #not exist media type
        channel_name = "error_media_type"
        media_type = 100
        CHANNEL_MANAGER.register_one_channel(channel_name)
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, PB_OPEN_CHANNEL_ERROR_OTHER)

        #clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        CHANNEL_MANAGER.unregister_one_channel(channel_name)
        server.stop_thread()


    def test_open_channel_fail_for_channel_busy(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        #channel busy
        channel_name = "image_busy"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = CHANNEL_MANAGER.register_one_channel(channel_name)
        self.assertEqual(ret, 0)
        handler = channel_handler.ChannelHandler(channel_name, media_type)
        CHANNEL_MANAGER.create_channel_resource(channel_name, 0, media_type, handler)
        CHANNEL_MANAGER.create_channel_resource(channel_name, 0, media_type, handler)
        ret = CHANNEL_MANAGER.is_channel_busy(channel_name)
        self.assertEqual(ret, True)

        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, PB_OPEN_CHANNEL_ERROR_CHANNEL_ALREADY_OPENED)

        #clean
        client.close()
        CHANNEL_MANAGER.unregister_one_channel(channel_name)
        server.stop_thread()

    def test_open_channel_with_protobuf_exception(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        #not exist channel name
        channel_name = "image"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel_with_protobuf_error(client)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, PB_OPEN_CHANNEL_ERROR_OTHER)

        #clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()

    def test_open_channel_with_decode_exception(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        #not exist channel name
        channel_name = "image"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel_with_encode_message_name_exception(client, channel_name, media_type)
        self.assertEqual(ret, True)

        #clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()


    def test_heartbeat(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        # test heartbeat
        channel_name = "heartbeat"
        media_type = PB_CHANNEL_CONTENT_TYPE_VIDEO
        CHANNEL_MANAGER.register_one_channel(channel_name)
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, True)

        ret = send_one_heartbeat_message(client)
        self.assertEqual(ret, True)

        # clean
        client.close()
        time.sleep(0.1)
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        CHANNEL_MANAGER.unregister_one_channel(channel_name)
        server.stop_thread()


    def test_not_recognize_message_type(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        # test heartbeat
        channel_name = "not_recognize_message_type"
        media_type = PB_CHANNEL_CONTENT_TYPE_VIDEO
        CHANNEL_MANAGER.register_one_channel(channel_name)
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, True)

        ret = send_one_not_recognize_message(client)
        self.assertEqual(ret, True)

        # clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        CHANNEL_MANAGER.unregister_one_channel(channel_name)
        server.stop_thread()

    def test_image_request_error(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)

        # test PRESENT_DATA_ERROR_UNSUPPORTED_FORMAT
        channel_name = "image_error"
        CHANNEL_MANAGER.register_one_channel(channel_name)
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, True)

        ret = send_one_image_message(client, 1)
        self.assertEqual(ret, True)

        ret = response_image_request(client)
        self.assertEqual(ret, PB_PRESENT_DATA_ERROR_UNSUPPORTED_FORMAT)

        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)

        # test PRESENT_DATA_ERROR_OTHER
        # have been clean by FaceDetectionServer(), reopen the channel
        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, True)

        for i in CHANNEL_MANAGER.channel_fds:
            CHANNEL_MANAGER.channel_fds[i].handler = None
        ret = send_one_image_message(client, None)
        self.assertEqual(ret, True)

        ret = response_image_request(client)
        self.assertEqual(ret, PB_PRESENT_DATA_ERROR_OTHER)
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)

        # test ParseFromString Exception
        # have been clean by FaceDetectionServer(), reopen the channel
        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, True)
        ret = send_one_image_message_protobuf_format_err(client)
        self.assertEqual(ret, True)

        ret = response_image_request(client)
        self.assertEqual(ret, PB_PRESENT_DATA_ERROR_OTHER)

        # clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        CHANNEL_MANAGER.unregister_one_channel(channel_name)
        server.stop_thread()

    @patch('select.epoll')
    def test_epollhup(self, mock_epoll):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)
        epoll = mock_epoll.return_value
        epoll.poll.side_effect = [[[0, select.EPOLLHUP]]]
        time.sleep(1)

        # clean
        server.stop_thread()


    @patch("socket.socket.recv")
    def test_read_socket(self, mock_socket_recv):
        """utest"""
        mock_socket_recv.side_effect = mock_recv
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        # test _read_socket
        ret, _ = server._read_socket(client, 10)
        self.assertEqual(ret, False)

        # clean
        client.close()
        server.stop_thread()

    @patch("socket.socket.recv")
    def test_read_msg_name(self, mock_socket_recv):
        """utest"""
        mock_socket_recv.side_effect = mock_recv
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        # prepare para
        fd = 1
        conns = {1:client}
        msg_body_len = 10

        # test _read_socket
        ret, _ = server._read_msg_name(fd, conns, msg_body_len)
        self.assertEqual(ret, False)

        # clean
        client.close()
        server.stop_thread()

    @patch("socket.socket.recv")
    def test_read_msg_body(self, mock_socket_recv):
        """utest"""
        mock_socket_recv.side_effect = mock_recv
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        # prepare para
        fd = 1
        conns = {1:client}
        msg_body_len = 10
        msgs = {}

        # test _read_socket
        ret = server._read_msg_body(fd, conns, msg_body_len, msgs)
        self.assertEqual(ret, False)

        # clean
        client.close()
        server.stop_thread()

    def test_receive_message_body_none(self):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        # test image
        channel_name = "image"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel_with_size_error(client, channel_name, media_type)
        self.assertEqual(ret, True)
        time.sleep(0.5)

        # clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()


    @patch("common.presenter_socket_server.PresenterSocketServer._read_msg_body")
    def test_read_msg_body_error(self, mock_read_msg_body):
        mock_read_msg_body.return_value = False
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        #test image
        channel_name = "image"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)

        #clean
        client.close()
        time.sleep(0.1)
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()




    @patch("common.presenter_socket_server.PresenterSocketServer._read_sock_and_process_msg")
    def test_socket_recv_error(self, mock_read_and_process_msg):
        """utest"""
        mock_read_and_process_msg.side_effect = socket_error

        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)

        client = create_sock_client(server_address)
        self.assertNotEqual(client, None)
        # test image
        channel_name = "image"
        media_type = PB_CHANNEL_CONTENT_TYPE_IMAGE
        ret = open_channel(client, channel_name, media_type)
        self.assertEqual(ret, True)
        ret = response_open_channel(client)
        self.assertEqual(ret, False)

        # clean
        client.close()
        CHANNEL_MANAGER.clean_channel_resource_by_name(channel_name)
        server.stop_thread()

    @patch('select.epoll')
    def test_receive_epoll_out(self, mock_epoll):
        """utest"""
        server_address = get_socket_server_addr()
        server = FaceDetectionServer(server_address)
        epoll = mock_epoll.return_value
        epoll.poll.side_effect = [[[0, select.EPOLLOUT]]]
        time.sleep(0.5)

        server.stop_thread()

if __name__ == '__main__':
    unittest.main()
    #suite = unittest.TestSuite()
    #suite.addTest(Test_PresenterSocketServer("test_image_request_error"))
    #runner = unittest.TextTestRunner()
    #runner.run(suite)
