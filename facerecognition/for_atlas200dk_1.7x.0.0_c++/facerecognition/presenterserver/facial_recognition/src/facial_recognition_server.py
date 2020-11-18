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
"""presenter facial recognition server module"""

import os
import json
import threading
import random
import logging
from logging.config import fileConfig
import numpy as np
from json.decoder import JSONDecodeError
from google.protobuf.message import DecodeError
import common.presenter_message_pb2 as presenter_message_pb2
from common.channel_manager import ChannelManager
from common.presenter_socket_server import PresenterSocketServer
from common.app_manager import AppManager
import facial_recognition.src.facial_recognition_message_pb2 as pb2
from facial_recognition.src.config_parser import ConfigParser
from facial_recognition.src.facial_recognition_handler import FacialRecognitionHandler


# Face Registration timeout is 10 seconds
FACE_REGISTER_TIME_OUT = 10

# Presenter Server Type
SERVER_TYPE = "facial_recognition"

# max app name length
APP_ID_MAX_LENGTH = 20

# max support 2 app connect
MAX_APP_NUM = 2

# length of face feature vector
FEATURE_VECTOR_LENGTH = 1024

# Face Registration Status code
FACE_REGISTER_STATUS_WAITING = 1
FACE_REGISTER_STATUS_SUCCEED = 2
FACE_REGISTER_STATUS_FAILED = 3

class FacialRecognitionServer(PresenterSocketServer):
    '''A server for face recognition'''
    def __init__(self, config):
        """
        Description: class init func
        Input:
            config: config information
        Returns: NA
        """
        server_address = (config.presenter_server_ip,
                          int(config.presenter_server_port))
        super(FacialRecognitionServer, self).__init__(server_address)
        self.storage_dir = config.storage_dir
        self.max_face_num = int(config.max_face_num)
        self.face_match_threshold = float(config.face_match_threshold)
        self.register_dict = {}
        self.app_manager = AppManager()
        self.channel_manager = ChannelManager()
        self.face_register_file = os.path.join(self.storage_dir,
                                               "registered_faces.json")
        self._init_face_database()

    def _init_face_database(self):
        """
        Description: Init face recognition database,
                     read information from face_register_file
        Input: NA
        Returns: NA
        """
        if not os.path.isfile(self.face_register_file):
            with open(self.face_register_file, "w", encoding="utf-8") as f:
                f.write("{}")

        with open(self.face_register_file, "r") as f:
            self.face_lock = threading.Lock()
            self.registered_faces = json.load(f)
            self._filter_registration_data()

    def _filter_registration_data(self):
        face_dict = self.registered_faces.copy()
        for i in face_dict:
            image_path = os.path.join(self.storage_dir, i + ".jpg")
            if not os.path.isfile(image_path):
                del self.registered_faces[i]

    def get_all_face(self):
        """
        Description: get registered face list.
        Input: NA
        Returns: NA
        """
        with self.face_lock:
            return [i for i in self.registered_faces]

    def save_face_image(self, name, image):
        """
        Description: save face image.
        Input:
            name face name
            image: face image
        Returns: True or False
        """
        image_file = os.path.join(self.storage_dir, name + ".jpg")
        try:
            #image = image.decode("utf-8")
            with open(image_file, "wb") as f:
                f.write(image)
            return True
        except (OSError, TypeError) as exp:
            logging.error(exp)
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

    def delete_faces(self, name_list):
        """
        Description: delete registered faces in name_list
        Input:
            name_list: a name list
        Returns: True or False
        """
        with self.face_lock:
            for i in name_list:
                if self.registered_faces.get(i):
                    backup = self.registered_faces[i]
                    del self.registered_faces[i]
                    try:
                        with open(self.face_register_file, "w") as f:
                            json.dump(self.registered_faces, f)
                        image_file = os.path.join(
                            self.storage_dir, i + ".jpg")
                        os.remove(image_file)
                    except (OSError, JSONDecodeError) as exp:
                        logging.error(exp)
                        self.registered_faces[i] = backup
                        return False
        return True

    def _clean_connect(self, sock_fileno, epoll, conns, msgs):
        """
        Description: close socket, and clean local variables
        Input:
            sock_fileno: a socket fileno, return value of socket.fileno()
            epoll: a set of select.epoll.
            conns: all socket connections registered in epoll
            msgs: msg read from a socket
        """
        logging.info("clean fd:%s, conns:%s", sock_fileno, conns)
        self.app_manager.unregister_app_by_fd(sock_fileno)
        epoll.unregister(sock_fileno)
        conns[sock_fileno].close()
        del conns[sock_fileno]
        del msgs[sock_fileno]


    def _process_msg(self, conn, msg_name, msg_data):
        """
        Total entrance to process protobuf msg
        Input:
            conn: a socket connection
            msg_name: name of a msg.
            msg_data: msg body, serialized by protobuf

        Returns:
            False:somme error occured
            True:succeed

        """
        # process open channel request
        if msg_name == pb2._REGISTERAPP.full_name:
            ret = self._process_register_app(conn, msg_data)
        # process image request, receive an image data from presenter agent
        elif msg_name == pb2._FACERESULT.full_name:
            ret = self._process_face_result(msg_data)
        elif msg_name == pb2._FRAMEINFO.full_name:
            ret = self._process_frame_info(conn, msg_data)
        elif msg_name == presenter_message_pb2._OPENCHANNELREQUEST.full_name:
            ret = self._process_open_channel(conn, msg_data)
        # process heartbeat request, it used to keepalive a channel path
        elif msg_name == presenter_message_pb2._HEARTBEATMESSAGE.full_name:
            ret = self._process_heartbeat(conn)
        else:
            logging.error("Not recognized msg type %s", msg_name)
            ret = False

        return ret

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
        request = pb2.RegisterApp()
        response = pb2.CommonResponse()
        msg_name = pb2._COMMONRESPONSE.full_name
        if not self._parse_protobuf(request, msg_data):
            response.ret = pb2.kErrorOther
            response.message = "ParseFromString exception"
            self.send_message(conn, response, msg_name)
            return False

        app_id = request.id
        app_type = request.type

        # check app id if exist
        if self.app_manager.is_app_exist(app_id):
            logging.error("App %s is already exist.", app_id)
            response.ret = pb2.kErrorAppRegisterExist
            response.message = "App {} is already exist.".format(app_id)
            self.send_message(conn, response, msg_name)
        elif self.app_manager.get_app_num() >= MAX_APP_NUM:
            logging.error("App number reach the upper limit")
            response.ret = pb2.kErrorAppRegisterLimit
            response.message = "App number reach the upper limit"
            self.send_message(conn, response, msg_name)
        elif app_type != SERVER_TYPE:
            logging.error("App type %s error", app_type)
            response.ret = pb2.kErrorAppRegisterType
            response.message = "App type {} error".format(app_type)
            self.send_message(conn, response, msg_name)
        elif len(app_id) > APP_ID_MAX_LENGTH:
            logging.error("App id %s is too long", app_id)
            response.ret = pb2.kErrorOther
            response.message = "App id: {} is too long".format(app_id)
            self.send_message(conn, response, msg_name)
        else:
            self.app_manager.register_app(app_id, conn)
            response.ret = pb2.kErrorNone
            response.message = "Register app {} succeed".format(app_id)
            self.send_message(conn, response, msg_name)
            return True

        return False

    def _process_face_result(self, msg_data):
        """
        Description: process face_result message
        Input:
            msg_data: msg body, serialized by protobuf
        Returns: True or False
        """
        face_result = pb2.FaceResult()
        if not self._parse_protobuf(face_result, msg_data):
            return False

        face_id = face_result.id
        if not self.register_dict.get(face_id):
            logging.warning("face id %s is already deleted", face_id)
            return True

        ret = face_result.response.ret
        if ret != pb2.kErrorNone:
            err_msg = face_result.response.message
            logging.error("get face feature error message: %s", err_msg)
            status = FACE_REGISTER_STATUS_FAILED
            message = "Get face feature failed"
            self._update_register_dict(face_id, status, message)
            return True

        face_num = len(face_result.feature)
        if face_num == 0:
            status = FACE_REGISTER_STATUS_FAILED
            message = "No face recognized"
            self._update_register_dict(face_id, status, message)
        elif face_num > 1:
            status = FACE_REGISTER_STATUS_FAILED
            message = "{} faces recognized".format(face_num)
            self._update_register_dict(face_id, status, message)
        else:
            box = face_result.feature[0].box
            face_coordinate = [box.lt_x, box.lt_y, box.rb_x, box.rb_x]
            feature_vector = [i for i in face_result.feature[0].vector]
            if len(feature_vector) != FEATURE_VECTOR_LENGTH:
                logging.error("feature_vector length not equal 1024")
                status = FACE_REGISTER_STATUS_FAILED
                message = "Face feature vector length invalid"
                self._update_register_dict(face_id, status, message)
                return True
            return self._save_face_feature(face_id, face_coordinate,
                                           feature_vector)

        return True

    def _update_register_dict(self, face_id, status, message):
        """
        Description: update register_dict
        Input:
            face_id: id of face
            status: status of face register
            message: message of status of face register
        Returns: True or False
        """
        if self.register_dict.get(face_id):
            self.register_dict[face_id]["status"] = status
            self.register_dict[face_id]["message"] = message
            self.register_dict[face_id]["event"].set()

    def _save_face_feature(self, face_id, face_coordinate, feature_vector):
        """
        Description: save face_feature
        Input:
            face_id: id of face
            face_coordinate: face coordinates
            feature_vector: face feature vector
        Returns: True or False
        """
        with self.face_lock:
            self.registered_faces[face_id] = {
                "coordinate": face_coordinate,
                "feature": feature_vector
            }
            try:
                with open(self.face_register_file, "w") as f:
                    json.dump(self.registered_faces, f)
                status = FACE_REGISTER_STATUS_SUCCEED
                message = "Successful registration"
                self._update_register_dict(face_id, status, message)
                return True
            except (OSError, JSONDecodeError) as exp:
                logging.error(exp)
                del self.registered_faces[face_id]
                status = FACE_REGISTER_STATUS_FAILED
                message = "save face feature to json file failed"
                self._update_register_dict(face_id, status, message)
                return False

    def _process_open_channel(self, conn, msg_data):
        """
        Description: process open channel message
        Input:
            conn: a socket connection
            msg_data: msg body, serialized by protobuf
        Returns: True or False
        """
        request = presenter_message_pb2.OpenChannelRequest()
        response = presenter_message_pb2.OpenChannelResponse()
        if not self._parse_protobuf(request, msg_data):
            channel_name = "unknown channel"
            err_code = presenter_message_pb2.kOpenChannelErrorOther
            return self._response_open_channel(conn, channel_name,
                                               response, err_code)
        channel_name = request.channel_name

        # check channel name if exist
        if not self.channel_manager.is_channel_exist(channel_name):
            logging.error("channel name %s is not exist.", channel_name)
            err_code = presenter_message_pb2.kOpenChannelErrorNoSuchChannel
            return self._response_open_channel(conn, channel_name,
                                               response, err_code)
            #ret = self.channel_manager.register_one_channel(channel_name)
            #if ret != ChannelManager.err_code_ok:
            #    logging.error("Create the channel %s failed!, and ret is %d", channel_name, ret)
            #    err_code =  pb2.kOpenChannelErrorOther
            #    self._response_open_channel(conn, channel_name, response, err_code)

        # check channel path if busy
        if self.channel_manager.is_channel_busy(channel_name):
            logging.error("channel path %s is busy.", channel_name)
            err = presenter_message_pb2.kOpenChannelErrorChannelAlreadyOpened
            return self._response_open_channel(conn, channel_name,
                                               response, err)

        content_type = presenter_message_pb2.kChannelContentTypeVideo
        if request.content_type == content_type:
            media_type = "video"
        else:
            logging.error("media type %s is not recognized.",
                          request.content_type)
            err_code = presenter_message_pb2.kOpenChannelErrorOther
            return self._response_open_channel(conn, channel_name,
                                               response, err_code)

        handler = FacialRecognitionHandler(channel_name, media_type)
        sock = conn.fileno()
        self.channel_manager.create_channel_resource(channel_name, sock,
                                                     media_type, handler)
        err_code = presenter_message_pb2.kOpenChannelErrorNone
        return self._response_open_channel(conn, channel_name,
                                           response, err_code)

    def _process_frame_info(self, conn, msg_data):
        """
        Description: process frame info message
        Input:
            conn: a socket connection
            msg_data: msg body, serialized by protobuf
        Returns: True or False
        """
        request = pb2.FrameInfo()
        response = pb2.CommonResponse()
        msg_name = pb2._COMMONRESPONSE.full_name
        if not self._parse_protobuf(request, msg_data):
            return False

        sock_fileno = conn.fileno()
        handler = self.channel_manager.get_channel_handler_by_fd(sock_fileno)
        if handler is None:
            logging.error("get channel handler failed")
            response.ret = pb2.kErrorOther
            response.message = "channel error."
            self.send_message(conn, response, msg_name)
            return False

        face_list = self._recognize_face(request.feature)
        handler.save_frame(request.image, face_list)
        response.ret = pb2.kErrorNone
        response.message = "process frame info suceed."
        self.send_message(conn, response, msg_name)
        return True

    def _recognize_face(self, face_feature):
        """
        Description:  recognize which face it is.
        Input:
            face_feature: face feature
        Returns: face list
        """
        face_list = []
        for i in face_feature:
            face_info = {}
            box = i.box
            coordinate = [box.lt_x, box.lt_y, box.rb_x, box.rb_y]
            feature_vector = i.vector
            if len(feature_vector) != FEATURE_VECTOR_LENGTH:
                logging.error("feature_vector length not equal 1024")
                continue

            (name, score) = self._compute_face_feature(feature_vector)
            face_info["coordinate"] = coordinate
            face_info["name"] = name
            face_info["confidence"] = score
            face_list.append(face_info)

        return face_list

    def _compute_face_feature(self, feture_vector):
        """
        Description: compute score of the feture_vector
        Input:
            feture_vector: face feature vector
        Returns: face name and score
        """
        highest_score_face = "Unknown"
        highest_score = 0
        with self.face_lock:
            for i in self.registered_faces:
                feature = self.registered_faces[i]["feature"]
                score = self._compute_similar_degree(feature, feture_vector)
                if score < self.face_match_threshold:
                    continue

                if score > highest_score:
                    highest_score = score
                    highest_score_face = i
        return (highest_score_face, highest_score)

    def _compute_similar_degree(self, feture_vector1, feture_vector2):
        """
        Description: compute cosine similarity of two vectors
        Input:
            feture_vector1: face feature vector
            feture_vector2: face feature vector
        Returns: score
        """
        vector1 = np.array(feture_vector1)
        vector2 = np.array(feture_vector2)
        square_diff = ((np.linalg.norm(vector1)) * (np.linalg.norm(vector2)))
        score = np.dot(vector1, vector2) / square_diff
        return score


    def stop_thread(self):
        """
        Description: clean thread when process exit.
        Input: NA
        Returns: NA
        """
        channel_manager = ChannelManager([])
        channel_manager.close_all_thread()
        self.set_exit_switch()
        self.app_manager.set_thread_switch()


class FacialRecognitionManager():
    '''Manager of Face Recognition, a class providing APIs'''
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

    def register_face(self, name, image):
        """
        Description: API for registering face
        Input:
            name: a face name
            image: a face picture
        Returns: (ret, msg)
        """

        # Input para check
        if not isinstance(name, str):
            return (False, "Name is not string")

        if not isinstance(image, bytes):
            return (False, "Image is not bytes")

        if self._get_face_number() >= self.server.max_face_num:
            return (False, "Face number limit")

        app_id = self._choose_random_app()
        if app_id is None:
            return (False, "No app is online")

        conn = self.server.get_app_socket(app_id)
        if conn is None:
            return (False, "Internal Error, app lost socket")

        # Prepare sending face register message to agent
        request = pb2.FaceInfo()
        request.id = name
        request.image = image

        register_dict = self.server.register_dict
        register_dict[name] = {
            "status": FACE_REGISTER_STATUS_WAITING,
            "message": "",
            "event": threading.Event()
        }

        msg_name = pb2._FACEINFO.full_name
        self.server.send_message(conn, request, msg_name)
        register_dict[name]["event"].wait(FACE_REGISTER_TIME_OUT)
        if register_dict[name]["status"] == FACE_REGISTER_STATUS_WAITING:
            logging.warning("Register face %s timeout", name)
            del register_dict[name]
            return (False, "10 sec Timeout")

        if register_dict[name]["status"] == FACE_REGISTER_STATUS_FAILED:
            err_msg = register_dict[name]["message"]
            logging.error("Register face %s failed, reason:%s",
                          name, register_dict[name]["message"])
            del register_dict[name]
            return (False, err_msg)

        ret = self.server.save_face_image(name, image)
        del register_dict[name]
        if ret:
            logging.info("Register face %s succeed", name)
            return (True, "Successful Registration")

        logging.error("Save face %s to database failed", name)
        return (False, "Save database error")

    def unregister_face(self, name_list):
        """
        Description: API for unregistering faces
        Input:
            name_list: a name list which will be deleted.
        Returns: True or False
        """
        if isinstance(name_list, list):
            return self.server.delete_faces(name_list)
        logging.error("unregister face fail")
        return False

    def get_all_face_name(self):
        """
        Description: API for geting all registered face names
        Input: NA
        Returns: a name list
        """
        return self.server.get_all_face()

    def _get_face_number(self):
        """
        Description: geting total face number
        Input: NA
        Returns: total face number
        """
        return len(self.get_all_face_name())

    def get_faces(self, name_list):
        """
        Description: API for geting specified face info.
        Input: a name list.
        Returns: a list include face name and image.
        """
        if not isinstance(name_list, list):
            return []

        face_list = []
        for i in name_list:
            face_info = {}
            face_info["name"] = i
            try:
                image_file = os.path.join(self.server.storage_dir, i + ".jpg")
                face_info["image"] = open(image_file, 'rb').read()
            except OSError as exp:
                logging.error(exp)
                continue
            face_list.append(face_info)

        return face_list

def run():
    '''Face Recognition server startup function'''
    # read config file
    config = ConfigParser()

    # config log
    log_file_path = os.path.join(ConfigParser.root_path, "config/logging.conf")
    fileConfig(log_file_path)
    logging.getLogger('facial_recognition')

    if not config.config_verify():
        return None

    server = FacialRecognitionServer(config)
    FacialRecognitionManager(server)
    return server
