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
"""
web application for presenter server.
"""
import os
import re
import random
import base64
import threading
import time
import logging
import tornado.ioloop
import tornado.web
import tornado.gen
import tornado.websocket
import common.channel_manager as channel_manager
import facial_recognition.src.config_parser as config_parser
import facial_recognition.src.facial_recognition_server as facial_recognition_server

# app
G_WEBAPP = None

# jpeg base64 header
JPEG_BASE64_HEADER = "data:image/jpeg;base64,"

# get request
REQUEST = "req"

# get appname
APP_NAME = "app_name"

# get username
USER_NAME = "user_name"

# get image
IMAGE = "image_data"

# return code
RET_CODE_SUCCESS = "0"
RET_CODE_FAIL = "1"
RET_CODE_LOADING = "2"


class WebApp:
    """
    web application
    """
    __instance = None
    def __init__(self):
        """
        init method
        """
        self.channel_mgr = channel_manager.ChannelManager()

        self.facial_recognize_manage = facial_recognition_server.FacialRecognitionManager()

        self.request_list = set()

        self.lock = threading.Lock()

        self.videostate = {"ret":0,"msg":{"name":"","req":0}}


    def __new__(cls, *args, **kwargs):
        # if instance is None than create one
        if cls.__instance is None:
            cls.__instance = object.__new__(cls, *args, **kwargs)
        return cls.__instance


    def get_videostate(self):
        '''get video state'''
        tmpvideostate = self.videostate
        tmplist = self.facial_recognize_manage.get_app_list()
        if tmpvideostate["ret"] == 1 and tmpvideostate["msg"]["name"] in tmplist:
            return self.videostate
        else:
            tmpvideostate["ret"] = 0
            self.videostate = tmpvideostate
            return tmpvideostate


    def list_registered_apps(self):
        '''get registered apps'''
        app_list = self.facial_recognize_manage.get_app_list()
        ret = []
        idx = 1
        for item in app_list:
            ret.append({"id":idx, "appname":item})
            idx = idx+1

        return ret


    def is_channel_exists(self, name):
        '''check is app exists'''
        return  self.channel_mgr.is_channel_exist(name)


    def register_face(self, user_name, image_data):
        ''' register user face'''
        ret = {"ret":RET_CODE_FAIL, "msg":""}

        if user_name is None:
            logging.info("User name is None, register face failed")
            ret["msg"] = "User name can not be empty"
            return ret

        #strip user name
        user_name = user_name.strip()

        if user_name == "":
            logging.info("User name is empty, register face failed")
            ret["msg"] = "User name can not be empty"
            return ret

        if len(user_name) > 50:
            logging.info("Length of User name %s > 50 , register face failed", user_name)
            ret["msg"] = "Length of User name should less than 50"
            return ret

        if image_data is None:
            logging.info("Image data is None, register face failed")
            ret["msg"] = "Image data can not be empty"
            return ret

        # define pattern support a-z A-Z and /
        pattern = re.compile(r"[a-z]|[A-Z]|[0-9]|(\s)")
        tmp = pattern.findall(user_name)

        # check reuslt changed or not
        if len(tmp) != len(user_name):
            logging.info("%s contain invalidate character, add channel failed", user_name)
            ret["msg"] = "Channel name only support 0-9, a-z, A-Z /"
            return ret

        # check image base64 code
        if len(image_data) <= len(JPEG_BASE64_HEADER):
            logging.info("Invalid jpeg base64 header identifier")
            ret["msg"] = "Just support image in jpg/jpeg format"
            return ret

        # jpeg base64 header check
        if image_data[0:len(JPEG_BASE64_HEADER)] != JPEG_BASE64_HEADER:
            logging.info("Invalid jpeg base64 header identifier")
            ret["msg"] = "Just support image in jpg/jpeg format"
            return ret

        # remove base64 header "data:image/jpeg;base64,""
        img_data = image_data[len(JPEG_BASE64_HEADER):len(image_data)]

        try:
            #convert to binary data
            decode_img = base64.b64decode(img_data)
        except (ValueError, TypeError) as exp:
            logging.error(exp)
            return {"ret":RET_CODE_FAIL, "msg":"Image decode error"}

        flag = self.facial_recognize_manage.register_face(user_name, decode_img)

        if flag[0] is True:
            logging.info("Register face success")
            ret = {"ret":RET_CODE_SUCCESS, "msg":flag[1]}
        else:
            logging.info("Register face failed")
            ret = {"ret":RET_CODE_FAIL, "msg":flag[1]}

        return ret

    def unregister_face(self, name_list):
        '''delete regeistered face'''
        ret = {"ret":RET_CODE_FAIL, "msg":""}

        if not name_list:
            logging.info("Name list is empty,delete name failed")
            ret["msg"] = "Name list should not be empty"
            return ret

        flag = self.facial_recognize_manage.unregister_face(name_list)

        if flag is False:
            ret["ret"] = RET_CODE_FAIL
            ret["msg"] = "Delete face failed"
            logging.info("Delete face failed")
        elif flag is True:
            ret["ret"] = RET_CODE_SUCCESS
            ret["msg"] = "Delete face success"
            logging.info("Delete face success")

        return ret


    def list_allface(self):
        '''list all users face'''
        name_list = self.facial_recognize_manage.get_all_face_name()

        if not name_list:
            return []

        name_list = sorted(name_list)
        show_face = self.facial_recognize_manage.get_faces(name_list)
        for item in show_face:
            try:
                #convert binary data to base64
                item["image"] = JPEG_BASE64_HEADER + base64.b64encode(item["image"]).decode("utf-8")
            except (ValueError, TypeError) as exp:
                logging.error(exp)
                return []

        return show_face

    def list_allfacename(self):
        '''list all register user name'''
        return self.facial_recognize_manage.get_all_face_name()


    # Input：appname
    # Output:
    # {
        # status: ok/error
        # image: image data
        # fps: frame per second
        # face_list: [{
        #     "name":value,
        #     "confidence":value
        #     "coordinate":[lt_x, lt_y, rb_x, rb_y]
        # }] }
    # }
    def get_media_data(self, app_name):
        '''get media data'''
        ret = {"ret":RET_CODE_FAIL, "image":"", "fps":"0", "face_list":""}

        if self.is_channel_exists(app_name) is False:
            return ret

        handler = self.channel_mgr.get_channel_handler_by_name(app_name)

        ret["ret"] = RET_CODE_LOADING

        if handler is not None:
            frame_info = handler.get_frame()
        else:
            return ret

        if not frame_info:
            return ret

        try:
            ret["image"] = base64.b64encode(frame_info["image"]).decode("utf-8")
        except (TypeError, ValueError) as exp:
            logging.error(exp)
            return ret

        ret["ret"] = RET_CODE_SUCCESS
        ret["fps"] = frame_info["fps"]
        ret["face_list"] = frame_info["face_list"]

        return ret

    def add_requst(self, request):
        """
        add request

        @param  requst: request item to be stored

        @note: request can not be same with other request.
               request is identified by   (channel name ,random number)
               so this method do not return value.
        """
        with self.lock:
            self.request_list.add(request)
            self.videostate = {"ret":1,"msg":{"name":request[1],"req":request[0]}}

    def has_request(self, request):
        """
        whether request exist or not

        @param  request:  request to be checked.
        @return:  return True if exists, otherwise return False.
        """
        with self.lock:

            for item in self.request_list:

                # check request equal
                if item[0] == request[0] and item[1] == request[1]:
                    return True

            return False

# pylint: disable=abstract-method
class BaseHandler(tornado.web.RequestHandler):
    """
    base handler.
    """

# pylint: disable=abstract-method
class ApplistHandler(BaseHandler):
    """
    handler index request
    """

    @tornado.web.asynchronous
    def get(self):
        """
        handle home or index request only for get
        """
        # self.render("applist.html", listret=G_WEBAPP.list_registered_apps())
        self.render("home.html", listret=(G_WEBAPP.list_registered_apps(), G_WEBAPP.list_allface(), G_WEBAPP.get_videostate()))

# pylint: disable=abstract-method
class RegisterHandler(BaseHandler):
    """
    handler register face
    """
    @tornado.web.asynchronous
    def post(self):
        """
        handle reqeust for register face
        """
        user_name = self.get_argument(USER_NAME, '')
        name_list = G_WEBAPP.list_allfacename()

        # check user name is duplicate
        for item in name_list:
            if user_name == item:
                self.finish({"ret":RET_CODE_FAIL, "msg":"user name has existed"})
                return None

        image_data = self.get_argument(IMAGE, '')

        self.finish(G_WEBAPP.register_face(user_name, image_data))
        return None


# pylint: disable=abstract-method
class DelFaceHandler(BaseHandler):
    """
    handler delete request
    """
    @tornado.web.asynchronous
    def post(self):
        """
        handel requst for delete channel
        """
        name_list = self.get_arguments(USER_NAME)

        self.finish(G_WEBAPP.unregister_face(name_list))


# pylint: disable=abstract-method
class ViewHandler(BaseHandler):
    """
    handler view request
    """
    @tornado.web.asynchronous
    def get(self):
        """
        handler request for view channel
        """
        channel_name = self.get_argument(APP_NAME, '')
        if G_WEBAPP.is_channel_exists(channel_name):
            req_id = str(random.random())
            G_WEBAPP.add_requst((req_id, channel_name))
            self.finish({"ret":RET_CODE_SUCCESS,"msg":req_id})
        else:
            self.finish({"ret":RET_CODE_FAIL,"msg":"Channel not exist"})

# pylint: disable=abstract-method

class WebSocket(tornado.websocket.WebSocketHandler):
    """
    web socket for web page socket quest
    """
    def open(self):
        """
        called when client request by ws or wss
        """

        self.req_id = self.get_argument(REQUEST, '', True)
        self.channel_name = self.get_argument(APP_NAME, '', True)

        # check request valid or not.
        if not G_WEBAPP.has_request((self.req_id, self.channel_name)):
            self.close()


    @staticmethod
    def send_message(obj, message, binary=False):
        """
        send message to client.
        """

        # check socket exist or not
        if not obj.ws_connection or not obj.ws_connection.stream.socket:
            return False

        ret = False
        try:
            obj.write_message(message, binary)
            ret = True
        except tornado.websocket.WebSocketClosedError:
            ret = False

        return ret


    def on_close(self):
        """
        called when closed web socket
        """

    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def on_message(self, message):
        """
         On recv message from client.
        """
        if message == "next":
            self.run_task()


    def run_task(self):
        """
        send image to client
        """

        # check channel valid
        if not G_WEBAPP.is_channel_exists(self.channel_name) or \
           not G_WEBAPP.has_request((self.req_id, self.channel_name)):
            self.close()
            return

        result = G_WEBAPP.get_media_data(self.channel_name)

        # sleep 100ms if status not ok for frequently query
        if result['ret'] != RET_CODE_SUCCESS:
            time.sleep(0.1)

        # if channel not exist close websocket.
        if result['ret'] == RET_CODE_FAIL:
            self.close()
        # send message to client
        else:
            # close websoket when send failed or for image channel.
            ret = WebSocket.send_message(self, result)


def get_webapp():
    """
    start web applicatioin
    """
    # get template file and static file path.
    templatepath = os.path.join(config_parser.ConfigParser.get_rootpath(), "ui/templates")
    staticfilepath = os.path.join(config_parser.ConfigParser.get_rootpath(), "ui/static")

    # create application object.
    app = tornado.web.Application(handlers=[(r"/", ApplistHandler),
                                            (r"/register", RegisterHandler),
                                            (r"/delete", DelFaceHandler),
                                            (r"/view", ViewHandler),
                                            (r"/static/(.*)",
                                             tornado.web.StaticFileHandler,
                                             {"path": staticfilepath}),
                                            (r"/websocket", WebSocket)],
                                  template_path=templatepath)

    # create server
    http_server = tornado.httpserver.HTTPServer(app)

    return http_server


def start_webapp():
    """
    start webapp
    """
    global G_WEBAPP
    G_WEBAPP = WebApp()

    http_server = get_webapp()
    config = config_parser.ConfigParser()
    http_server.listen(config.web_server_port, address=config.web_server_ip)

    print("Please visit http://" + config.web_server_ip + ":" +
          str(config.web_server_port) + " for presenter server")
    tornado.ioloop.IOLoop.instance().start()


def stop_webapp():
    """
    stop web app
    """
    tornado.ioloop.IOLoop.instance().stop()
