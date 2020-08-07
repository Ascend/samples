# !/usr/bin/env python
# -*- coding:utf-8 -*-
import struct
import socket

from . import presenter_message_pb2 as pb2

def pack_message(msg_name, msg_data):
    buf = msg_data.SerializeToString()
    msg_body_len = len(buf)
    msg_name_len = len(msg_name)
    msg_total_len = msg_name_len + msg_body_len + 5
    data = b''
    msg_total_len = socket.htonl(msg_total_len)
    pack_data = struct.pack('IB', msg_total_len, msg_name_len)
    data += pack_data
    data += msg_name.encode()
    data += buf

    return data

def open_channel_request(channel_name, content_type):
    request = pb2.OpenChannelRequest()
    request.channel_name = channel_name
    request.content_type = content_type

    return pack_message(pb2._OPENCHANNELREQUEST.full_name, request)

def image_frame_request(image_width, image_height, image_data, detection_result):
    request = pb2.PresentImageRequest()
    request.format = 0
    request.width = image_width
    request.height = image_height
    request.data = image_data
    for i in range(0, len(detection_result)):
        myadd = request.rectangle_list.add()
        myadd.left_top.x = detection_result[i].box.lt.x
        myadd.left_top.y = detection_result[i].box.lt.y
        myadd.right_bottom.x = detection_result[i].box.rb.x
        myadd.right_bottom.y = detection_result[i].box.rb.y
        myadd.label_text = detection_result[i].result_text

    return pack_message(pb2._PRESENTIMAGEREQUEST.full_name, request)

def heartbeat_message():
    return pack_message(pb2._HEARTBEATMESSAGE.full_name, pb2.HeartbeatMessage())

def is_open_channel_response(msg_name):
    return (msg_name == pb2._OPENCHANNELRESPONSE.full_name)

def is_image_frame_response(msg_name):
    return (msg_name == pb2._PRESENTIMAGERESPONSE.full_name)




