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

"""presenter socket server module"""

import os
import logging
from logging.config import fileConfig
from google.protobuf.message import DecodeError
import common.presenter_message_pb2 as pb2
from common.channel_manager import ChannelManager
from common.presenter_socket_server import PresenterSocketServer
from common.channel_handler import ChannelHandler
from display.src.config_parser import ConfigParser

import random
from common.app_manager import AppManager
import display.src.painting_message_pb2 as painting_message_pb2


# Presenter Server Type
SERVER_TYPE = "display"

# max app name length
APP_ID_MAX_LENGTH = 20

# max support 2 app connect
MAX_APP_NUM = 2


class DisplayServer(PresenterSocketServer):
    '''A server for face detection'''

    def __init__(self, server_address):
        '''init func'''
        self.channel_manager = ChannelManager(["image", "video"])
        super(DisplayServer, self).__init__(server_address)

        self.app_manager = AppManager()
        self.register_dict = {}

    def _clean_connect(self, sock_fileno, epoll, conns, msgs):
        """
        close socket, and clean local variables
        Args:
            sock_fileno: a socket fileno, return value of socket.fileno()
            epoll: a set of select.epoll.
            conns: all socket connections registered in epoll
            msgs: msg read from a socket
        """
        logging.info("clean fd:%s, conns:%s", sock_fileno, conns)
        self.channel_manager.clean_channel_resource_by_fd(sock_fileno)
        epoll.unregister(sock_fileno)
        conns[sock_fileno].close()
        del conns[sock_fileno]
        del msgs[sock_fileno]

    def _process_msg(self, conn, msg_name, msg_data):
        """
        Total entrance to process protobuf msg
        Args:
            conn: a socket connection
            msg_name: name of a msg.
            msg_data: msg body, serialized by protobuf

        Returns:
            False:somme error occured
            True:succeed

        """
        ## process_register_app
        if msg_name == painting_message_pb2._REGISTERAPP.full_name:
            ret = self._process_register_app(conn, msg_data)
        # process open channel request
        elif msg_name == pb2._OPENCHANNELREQUEST.full_name:
            ret = self._process_open_channel(conn, msg_data)
        # process image request, receive an image data from presenter agent
        elif msg_name == pb2._PRESENTIMAGEREQUEST.full_name:
            ret = self._process_image_request(conn, msg_data)
        # process heartbeat request, it used to keepalive a channel path
        elif msg_name == pb2._HEARTBEATMESSAGE.full_name:
            ret = self._process_heartbeat(conn)
        else:
            logging.error("Not recognized msg type %s", msg_name)
            ret = False

        return ret

    # 重载_process_heartbeat
    def _process_heartbeat(self, conn):
        '''
        set heartbeat
        Input:
            conn: a socket connection
        Returns:
            True: set heartbeat ok.

        '''
        sock_fileno = conn.fileno()
        if self.app_manager.get_app_id_by_socket(sock_fileno):
            self.app_manager.set_heartbeat(sock_fileno)

        handler = self.channel_manager.get_channel_handler_by_fd(sock_fileno)
        if handler is not None:
            handler.set_heartbeat()
        return True

    def _parse_protobuf(self, protobuf, msg_data):
        """
        Description:  parse protobuf
        Input:
            protobuf: a struct defined by protobuf
            msg_data: msg body, serialized by protobuf
        Returns: True or False
        """
        try:
            protobuf.ParseFromString(msg_data)
            return True
        except DecodeError as exp:
            logging.error(exp)
            return False

    def _process_register_app(self, conn, msg_data):
        """
        Description: process register_app message
        Input:
            conn: a socket connection
            msg_data: msg body, serialized by protobuf
        Returns: True or False
        """
        request = painting_message_pb2.RegisterApp()
        response = painting_message_pb2.CommonResponse()
        msg_name = painting_message_pb2._COMMONRESPONSE.full_name
        if not self._parse_protobuf(request, msg_data):
            response.ret = painting_message_pb2.kErrorOther
            response.message = "ParseFromString exception"
            self.send_message(conn, response, msg_name)
            return False

        app_id = request.id
        app_type = request.type

        # check app id if exist
        if self.app_manager.is_app_exist(app_id):
            logging.error("App %s is already exist.", app_id)
            response.ret = painting_message_pb2.kErrorAppRegisterExist
            response.message = "App {} is already exist.".format(app_id)
            self.send_message(conn, response, msg_name)
        elif self.app_manager.get_app_num() >= MAX_APP_NUM:
            logging.error("App number reach the upper limit")
            response.ret = painting_message_pb2.kErrorAppRegisterLimit
            response.message = "App number reach the upper limit"
            self.send_message(conn, response, msg_name)
        elif app_type != SERVER_TYPE:
            logging.error("App type %s error", app_type)
            response.ret = painting_message_pb2.kErrorAppRegisterType
            response.message = "App type {} error".format(app_type)
            self.send_message(conn, response, msg_name)
        elif len(app_id) > APP_ID_MAX_LENGTH:
            logging.error("App id %s is too long", app_id)
            response.ret = painting_message_pb2.kErrorOther
            response.message = "App id: {} is too long".format(app_id)
            self.send_message(conn, response, msg_name)
        else:
            self.app_manager.register_app(app_id, conn)
            response.ret = painting_message_pb2.kErrorNone
            response.message = "Register app {} succeed".format(app_id)
            self.send_message(conn, response, msg_name)
            return True

        return False

    def get_app_socket(self, app_id):
        """
        Description: get a socket which is bound to the app.
        Input:
            app_id: id of the app
        Returns: socket
        """
        return self.app_manager.get_socket_by_app_id(app_id)

    def list_registered_apps(self):
        """
        Description: get registered apps list.
        Input: NA
        Returns: app list
        """
        return self.app_manager.list_app()

    def _response_image_request(self, conn, response, err_code):
        """
        Assemble protobuf to response image_request
        Message structure like this:
        --------------------------------------------------------------------
        |total message len   |    int         |    4 bytes                  |
        |-------------------------------------------------------------------
        |message name len    |    byte        |    1 byte                   |
        |-------------------------------------------------------------------
        |message name        |    string      |    xx bytes                 |
        |-------------------------------------------------------------------
        |message body        |    protobuf    |    xx bytes                 |
        --------------------------------------------------------------------

        protobuf structure like this:
        --------------------------------------------------------------------
        |error_code       |    enum          |    PresentDataErrorCode     |
        |-------------------------------------------------------------------
        |error_message    |    string        |    xx bytes                 |
        |-------------------------------------------------------------------

        enum PresentDataErrorCode {
            kPresentDataErrorNone = 0;
            kPresentDataErrorUnsupportedType = 1;
            kPresentDataErrorUnsupportedFormat = 2;
            kPresentDataErrorOther = -1;
        }
        """
        response.error_code = err_code
        ret_code = True
        if err_code == pb2.kPresentDataErrorUnsupportedFormat:
            response.error_message = "Present data not support format."
            logging.error("Present data not support format.")
            ret_code = False
        elif err_code == pb2.kPresentDataErrorNone:
            response.error_message = "Present data ok"
            ret_code = True
        else:
            response.error_message = "Present data not known error."
            logging.error("Present data not known error.")
            ret_code = False

        self.send_message(conn, response, pb2._PRESENTIMAGERESPONSE.full_name)
        return ret_code

    def _process_image_request(self, conn, msg_data):
        """
        Deserialization protobuf and process display image request
        Args:
            conn: a socket connection
            msg_data: a protobuf struct, include image request.

        Returns:

        protobuf structure like this:
         ------------------------------------
        |data          |    byts             |
        |------------------------------------
        |width         |    uint32           |
        |------------------------------------
        enum ImageFormat {
            kImageFormatJpeg = 0;
        }
        """
        request = pb2.PresentImageRequest()
        response = pb2.PresentImageResponse()

        # Parse msg_data from protobuf
        try:
            request.ParseFromString(msg_data)
        except DecodeError:
            logging.error("ParseFromString exception: Error parsing message")
            err_code = pb2.kPresentDataErrorOther
            return self._response_image_request(conn, response, err_code)

        sock_fileno = conn.fileno()
        handler = self.channel_manager.get_channel_handler_by_fd(sock_fileno)
        if handler is None:
            logging.error("get channel handler failed")
            err_code = pb2.kPresentDataErrorOther
            return self._response_image_request(conn, response, err_code)

        rectangle_list = []
        if request.rectangle_list:
            for one_rectangle in request.rectangle_list:
                rectangle = []
                rectangle.append(one_rectangle.left_top.x)
                rectangle.append(one_rectangle.left_top.y)
                rectangle.append(one_rectangle.right_bottom.x)
                rectangle.append(one_rectangle.right_bottom.y)
                rectangle.append(one_rectangle.label_text)
                # add the detection result to list
                rectangle_list.append(rectangle)

        handler.save_image(request.data, request.width, request.height, rectangle_list)
        return self._response_image_request(conn, response,
                                            pb2.kPresentDataErrorNone)

    def stop_thread(self):
        channel_manager = ChannelManager([])
        channel_manager.close_all_thread()
        self.set_exit_switch()

class DisplayServerManager():
    '''Manager of Display Server, a class providing APIs'''
    __instance = None
    server = None

    def __init__(self, server=None):
        '''init func'''

    def __new__(cls, server=None):
        """ensure only a single instance created. """
        if cls.__instance is None:
            cls.__instance = object.__new__(cls)
            cls.server = server
        return cls.__instance

    def _choose_random_app(self):
        """
        Description: choose a random app online.
        Input: NA
        Returns: a app name
        """
        app_list = self.server.list_registered_apps()
        print("app list:", app_list)
        if app_list:
            index = random.randint(0, len(app_list) - 1)
            return app_list[index]
        return None

    def get_app_list(self):
        """
        Description: API for getting online app list
        Input: NA
        Returns: app list
        """
        return self.server.list_registered_apps()

    def send_model_input_data(self, object_data, layout_data):
        """
        Description: API for send model input data
        Input:
            style_id: a style id, string type
        Returns: (ret, msg)
        """
        # Input para check
        if not isinstance(object_data, bytes):
            return (False, "object data is not bytes")
        if not isinstance(layout_data, bytes):
            return (False, "layout data is not bytes")

        app_id = self._choose_random_app()
        if app_id is None:
            print("No app is online.")
            return (False, "No app is online")

        conn = self.server.get_app_socket(app_id)
        if conn is None:
            print("Internal Error, app lost socket.")
            return (False, "Internal Error, app lost socket")

        ### Prepare sending data package to agent
        request = painting_message_pb2.DataPackage()
        request.objectData.name = 'object_data'
        request.objectData.data = object_data
        request.layoutData.name = 'layout_data'
        request.layoutData.data = layout_data

        msg_name = painting_message_pb2._DATAPACKAGE.full_name
        self.server.send_message(conn, request, msg_name)

        logging.info("Send model input data package succeed")
        return (True, "Successful")


def run():
    '''Entrance function of Face Detection Server '''
    # read config file
    config = ConfigParser()

    # config log
    log_file_path = os.path.join(ConfigParser.root_path, "config/logging.conf")
    fileConfig(log_file_path)
    logging.getLogger('display_server')

    if not config.config_verify():
        return None

    logging.info("presenter server is starting...")
    server_address = (config.presenter_server_ip,
                      int(config.presenter_server_port))
    server = DisplayServer(server_address)
    DisplayServerManager(server)
    return server
