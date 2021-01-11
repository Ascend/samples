"""utest presenter server module"""

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
import shutil
import unittest
from unittest.mock import patch
path = os.path.dirname(__file__)
index = path.rfind("ascenddk")
workspace = path[0: index]
path = os.path.join(workspace, "ascenddk/common/presenter/server/")
sys.path.append(path)

from face_detection.src.config_parser import ConfigParser
import presenter_server

class TestPresenterServer(unittest.TestCase):
    @patch('sys.exit', return_value=True)
    @patch('argparse.ArgumentParser.parse_args')
    @patch('face_detection.src.web.start_webapp', return_value=True)
    @patch('presenter_server.close_all_thread', return_value=True)
    @patch('common.presenter_socket_server.PresenterSocketServer._create_socket_server', return_value=True)
    def test_face_detection(self, mock1, mock2, mock3, mock4, mock5):
        """test_main"""
        args = mock4.return_value
        args.app = "face_detection"
        config_parser = ConfigParser()
        presenter_server.main_process()
        presenter_server.close_all_thread(None, None)

        # test_invalid_ip_and_port
        ConfigParser.presenter_server_ip = '0.0.0.0'
        ConfigParser.presenter_server_port = 0
        presenter_server.main_process()
        presenter_server.close_all_thread(None, None)

        # clean log
        os.system("rm -f *.log")


    @patch('sys.exit', return_value=True)
    @patch('argparse.ArgumentParser.parse_args')
    @patch('facial_recognition.src.web.start_webapp', return_value=True)
    @patch('presenter_server.close_all_thread', return_value=True)
    @patch('common.presenter_socket_server.PresenterSocketServer._create_socket_server', return_value=True)
    def test_face_recognition(self, mock1, mock2, mock3, mock4, mock5):
        """test_main"""
        args = mock4.return_value
        args.app = "facial_recognition"
        config_parser = ConfigParser()
        presenter_server.main_process()
        presenter_server.close_all_thread(None, None)

        # clean log
        os.system("rm -f *.log")

    @patch('sys.exit', return_value=True)
    @patch('argparse.ArgumentParser.parse_args')
    @patch('video_analysis.src.web.start_webapp', return_value=True)
    @patch('video_analysis.src.web.stop_webapp', return_value=True)
    @patch('video_analysis.src.video_analysis_server.VideoAnalysisServer')
    @patch('common.presenter_socket_server.PresenterSocketServer._create_socket_server', return_value=True)
    def test_video_analysis(self, mock1, mock2, mock3, mock4, mock5, mock6):
        """test_main"""
        video_server = mock2.return_value
        args = mock5.return_value
        args.app = "video_analysis"
        config_parser = ConfigParser()
        presenter_server.main_process()
        presenter_server.WEB_SERVER =  __import__("video_analysis.src.web", fromlist=True)
        presenter_server.RUN_SERVER = video_server
        video_server.stop_thread.return_value = True
        presenter_server.close_all_thread(None, None)

        # clean log
        os.system("rm -f *.log")

    @patch('sys.exit', return_value=True)
    @patch('argparse.ArgumentParser.parse_args')
    @patch('video_analysis.src.video_analysis_server.run', return_value=True)
    @patch('video_analysis.src.web.start_webapp', return_value=True)
    @patch('common.presenter_socket_server.PresenterSocketServer._create_socket_server', return_value=True)
    def test_main_process(self, mock1, mock2, mock3, mock4, mock5):
        """test_main"""
        args = mock4.return_value
        args.app = "video_analysis"
        mock3.return_value = None
        ret = presenter_server.main_process()
        self.assertEqual(ret, True)
        # clean log
        os.system("rm -f *.log")

    @patch('presenter_server.arg_parse', return_value=True)
    @patch('presenter_server.check_server_exist', return_value=0)
    def test_main_process_server_exist(self, mock1, mock2):
        """test_main"""
        ret = presenter_server.main_process()
        self.assertEqual(ret, True)
        # clean log
        os.system("rm -f *.log")



if __name__ == '__main__':
    unittest.main()
    #suite = unittest.TestSuite()
    #suite.addTest(Test_PresenterSocketServer("test_image_request_error"))
    #runner = unittest.TextTestRunner()
    #runner.run(suite)
